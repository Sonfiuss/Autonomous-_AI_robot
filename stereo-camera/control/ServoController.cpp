#include "ServoController.hpp"

#include <cstdio>
#include <cstring>
#include <cerrno>
#include <chrono>
#include <cstdlib>

// Set SERVO_DEBUG=1 in environment to enable verbose logging.
static bool dbg() {
    static int v = -1;
    if (v < 0) { const char* e = getenv("SERVO_DEBUG"); v = (e && e[0] == '1') ? 1 : 0; }
    return v == 1;
}
#define DBG(...) do { if (dbg()) fprintf(stderr, "[DBG] " __VA_ARGS__); } while(0)

// ── Port configuration ────────────────────────────────────────────────────────

bool ServoController::configurePort(int fd, int baud) {
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) return false;

    speed_t speed = B115200;
    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default:     speed = B115200; break;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_iflag  = IGNBRK;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag  = 0;
    tty.c_oflag  = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;   // 100 ms read timeout

    tcflush(fd, TCIFLUSH);
    return tcsetattr(fd, TCSANOW, &tty) == 0;
}

// ── Constructor / Destructor ──────────────────────────────────────────────────

ServoController::ServoController(const std::string& port, int baud)
    : m_port(port), m_baud(baud) {}

ServoController::~ServoController() {
    disconnect();
}

// ── Public API ────────────────────────────────────────────────────────────────

bool ServoController::connect(double timeout_s) {
    m_fd = open(m_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (m_fd < 0) {
        fprintf(stderr, "[ServoController] Cannot open %s: %s\n",
                m_port.c_str(), strerror(errno));
        return false;
    }
    if (!configurePort(m_fd, m_baud)) {
        fprintf(stderr, "[ServoController] Failed to configure port\n");
        close(m_fd);
        m_fd = -1;
        return false;
    }

    DBG("port opened: fd=%d baud=%d\n", m_fd, m_baud);
    m_running = true;
    m_rx_thread = std::thread(&ServoController::rxLoop, this);

    {
        std::unique_lock<std::mutex> lk(m_ready_mtx);
        auto deadline = std::chrono::steady_clock::now()
                      + std::chrono::duration<double>(timeout_s);
        if (!m_ready_cv.wait_until(lk, deadline, [this]{ return m_ready; }))
            fprintf(stderr, "[ServoController] Timeout waiting for READY "
                            "(ESP32 may already be running)\n");
    }

    printf("[ServoController] Connected to %s\n", m_port.c_str());
    return true;
}

void ServoController::disconnect() {
    m_running = false;
    if (m_rx_thread.joinable())
        m_rx_thread.join();
    if (m_fd >= 0) {
        close(m_fd);
        m_fd = -1;
    }
}

bool ServoController::moveTo(float pan_deg, float tilt_deg, double timeout_s) {
    char buf[64];
    snprintf(buf, sizeof(buf), "MOVE %.3f %.3f\n", pan_deg, tilt_deg);
    return sendAndWaitOK(buf, timeout_s);
}

void ServoController::stop() {
    send("STOP\n");
}

bool ServoController::reset(double timeout_s) {
    return sendAndWaitOK("RESET\n", timeout_s);
}

bool ServoController::isConnected() const {
    return m_fd >= 0 && m_running.load();
}

// ── Internal ──────────────────────────────────────────────────────────────────

bool ServoController::sendAndWaitOK(const std::string& cmd, double timeout_s) {
    {
        std::lock_guard<std::mutex> lk(m_ok_mtx);
        m_ok = false;   // clear before sending to avoid stale OK race
    }
    send(cmd);

    std::unique_lock<std::mutex> lk(m_ok_mtx);
    auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::duration<double>(timeout_s);
    bool got = m_ok_cv.wait_until(lk, deadline, [this]{ return m_ok; });
    if (!got) {
        std::string disp = cmd;
        if (!disp.empty() && disp.back() == '\n') disp.pop_back();
        fprintf(stderr, "[ServoController] Timeout waiting for OK (cmd: %s)\n",
                disp.c_str());
    }
    return got;
}

void ServoController::send(const std::string& msg) {
    if (m_fd < 0) return;
    ssize_t n = write(m_fd, msg.c_str(), msg.size());
    if (n < 0)
        fprintf(stderr, "[ServoController] Write error: %s\n", strerror(errno));
    else {
        std::string disp = msg;
        if (!disp.empty() && disp.back() == '\n') disp.pop_back();
        DBG("TX (%zd bytes): \"%s\"\n", n, disp.c_str());
    }
}

void ServoController::rxLoop() {
    std::string buf;
    char ch;
    DBG("rxLoop started\n");
    while (m_running) {
        ssize_t n = read(m_fd, &ch, 1);
        if (n < 0) {
            DBG("RX read error: %s\n", strerror(errno));
            continue;
        }
        if (n == 0) continue;   // VTIME timeout, no data
        if (ch == '\n') {
            if (!buf.empty() && buf.back() == '\r') buf.pop_back();
            if (!buf.empty()) {
                DBG("RX line: \"%s\"\n", buf.c_str());
                parseLine(buf);
            }
            buf.clear();
        } else {
            buf += ch;
        }
    }
    DBG("rxLoop exiting\n");
}

void ServoController::parseLine(const std::string& line) {
    if (line == "READY") {
        DBG("parseLine: READY\n");
        {
            std::lock_guard<std::mutex> lk(m_ready_mtx);
            m_ready = true;
        }
        m_ready_cv.notify_all();
        return;
    }
    if (line == "OK") {
        DBG("parseLine: OK — signalling\n");
        {
            std::lock_guard<std::mutex> lk(m_ok_mtx);
            m_ok = true;
        }
        m_ok_cv.notify_all();
        return;
    }
    // Log anything unrecognised (e.g. P/O/K from unified controller)
    DBG("parseLine: ignored \"%s\"\n", line.c_str());
}
