#include "ServoClient.hpp"

#include <cstring>
#include <cstdio>
#include <stdexcept>
#include <sstream>
#include <chrono>
#include <cerrno>

// ── Helpers ───────────────────────────────────────────────────────────────────

double ServoClient::mono_now() {
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

bool ServoClient::configurePort(int fd, int baud) {
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

    // 8N1, no flow control
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_iflag  = IGNBRK;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag  = 0;
    tty.c_oflag  = 0;

    // Read timeout: 100 ms
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    tcflush(fd, TCIFLUSH);
    return tcsetattr(fd, TCSANOW, &tty) == 0;
}

// ── Constructor / Destructor ──────────────────────────────────────────────────

ServoClient::ServoClient(const std::string& port, int baud)
    : m_port(port), m_baud(baud) {}

ServoClient::~ServoClient() {
    disconnect();
}

// ── Public API ────────────────────────────────────────────────────────────────

bool ServoClient::connect(double timeout_s) {
    m_fd = open(m_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (m_fd < 0) {
        fprintf(stderr, "[ServoClient] Cannot open %s: %s\n",
                m_port.c_str(), strerror(errno));
        return false;
    }

    if (!configurePort(m_fd, m_baud)) {
        fprintf(stderr, "[ServoClient] Failed to configure port\n");
        close(m_fd);
        m_fd = -1;
        return false;
    }

    m_running = true;
    m_rx_thread = std::thread(&ServoClient::rxLoop, this);

    // Wait for "READY"
    {
        std::unique_lock<std::mutex> lk(m_ready_mutex);
        auto deadline = std::chrono::steady_clock::now()
                      + std::chrono::duration<double>(timeout_s);
        if (!m_ready_cv.wait_until(lk, deadline, [this]{ return m_ready; })) {
            fprintf(stderr, "[ServoClient] Timeout waiting for READY\n");
            // Don't fail hard – ESP32 may already be running
        }
    }

    printf("[ServoClient] Connected to %s\n", m_port.c_str());
    return true;
}

void ServoClient::disconnect() {
    m_running = false;
    if (m_rx_thread.joinable())
        m_rx_thread.join();
    if (m_fd >= 0) {
        close(m_fd);
        m_fd = -1;
    }
}

void ServoClient::setVelocity(float pan_vel, float tilt_vel) {
    char buf[64];
    snprintf(buf, sizeof(buf), "V %.3f %.3f\n", pan_vel, tilt_vel);
    send(buf);
}

void ServoClient::stop() {
    send("S\n");
}

void ServoClient::reset() {
    send("R\n");
}

ServoClient::State ServoClient::getState() const {
    std::lock_guard<std::mutex> lk(m_state_mutex);
    return m_state;
}

void ServoClient::setPositionCallback(PositionCallback cb) {
    std::lock_guard<std::mutex> lk(m_cb_mutex);
    m_callback = std::move(cb);
}

// ── Internal ──────────────────────────────────────────────────────────────────

void ServoClient::send(const std::string& msg) {
    if (m_fd < 0) return;
    ssize_t written = write(m_fd, msg.c_str(), msg.size());
    if (written < 0)
        fprintf(stderr, "[ServoClient] Write error: %s\n", strerror(errno));
}

void ServoClient::rxLoop() {
    std::string buf;
    char ch;

    while (m_running) {
        ssize_t n = read(m_fd, &ch, 1);
        if (n <= 0) continue;

        if (ch == '\n') {
            // Strip trailing '\r' if present
            if (!buf.empty() && buf.back() == '\r') buf.pop_back();
            if (!buf.empty()) parseLine(buf);
            buf.clear();
        } else {
            buf += ch;
        }
    }
}

void ServoClient::parseLine(const std::string& line) {
    if (line == "READY") {
        {
            std::lock_guard<std::mutex> lk(m_ready_mutex);
            m_ready = true;
        }
        m_ready_cv.notify_all();
        return;
    }

    // "P <pan> <tilt>"
    if (line.size() >= 3 && line[0] == 'P' && line[1] == ' ') {
        float pan = 0.f, tilt = 0.f;
        if (sscanf(line.c_str() + 2, "%f %f", &pan, &tilt) == 2) {
            State s;
            s.pan       = pan;
            s.tilt      = tilt;
            s.timestamp = mono_now();

            {
                std::lock_guard<std::mutex> lk(m_state_mutex);
                m_state = s;
            }

            PositionCallback cb;
            {
                std::lock_guard<std::mutex> lk(m_cb_mutex);
                cb = m_callback;
            }
            if (cb) cb(s);
        }
    }
}
