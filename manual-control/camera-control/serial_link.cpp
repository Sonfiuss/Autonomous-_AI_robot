#include "serial_link.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <sstream>

SerialLink::SerialLink(std::string port, int baud)
    : m_port(std::move(port)), m_baud(baud) {}

SerialLink::~SerialLink() { disconnect(); }

bool SerialLink::connect(double timeout_s) {
    m_fd = open(m_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (m_fd < 0) {
        fprintf(stderr, "Cannot open serial port %s: %s\n", m_port.c_str(), strerror(errno));
        return false;
    }
    if (!configurePort()) {
        close(m_fd);
        m_fd = -1;
        return false;
    }

    m_running = true;
    m_rx = std::thread(&SerialLink::rxLoop, this);

    // Wait for the ESP32 boot "READY" line.
    std::unique_lock<std::mutex> lk(m_ready_mtx);
    bool ok = m_ready_cv.wait_for(lk, std::chrono::duration<double>(timeout_s),
                                  [this] { return m_ready; });
    if (!ok)
        fprintf(stderr, "Warning: no READY from ESP32 within %.1fs (continuing)\n", timeout_s);
    return true;
}

void SerialLink::disconnect() {
    if (m_running.exchange(false)) {
        if (m_rx.joinable()) m_rx.join();
    }
    if (m_fd >= 0) {
        close(m_fd);
        m_fd = -1;
    }
}

bool SerialLink::configurePort() {
    struct termios tty{};
    if (tcgetattr(m_fd, &tty) != 0) {
        fprintf(stderr, "tcgetattr failed: %s\n", strerror(errno));
        return false;
    }

    speed_t spd = (m_baud == 115200) ? B115200 :
                  (m_baud == 57600)  ? B57600  :
                  (m_baud == 9600)   ? B9600   : B115200;
    cfsetospeed(&tty, spd);
    cfsetispeed(&tty, spd);

    tty.c_cflag &= ~PARENB;            // no parity
    tty.c_cflag &= ~CSTOPB;            // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                // 8 data bits
    tty.c_cflag &= ~CRTSCTS;           // no HW flow control
    tty.c_cflag |= CREAD | CLOCAL;     // enable read, ignore modem lines

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);   // raw input
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
    tty.c_oflag &= ~OPOST;             // raw output

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(m_fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "tcsetattr failed: %s\n", strerror(errno));
        return false;
    }
    tcflush(m_fd, TCIOFLUSH);
    return true;
}

void SerialLink::rxLoop() {
    std::string buf;
    char chunk[256];
    while (m_running) {
        ssize_t n = read(m_fd, chunk, sizeof(chunk));
        if (n > 0) {
            buf.append(chunk, n);
            size_t nl;
            while ((nl = buf.find('\n')) != std::string::npos) {
                std::string line = buf.substr(0, nl);
                if (!line.empty() && line.back() == '\r') line.pop_back();
                if (!line.empty()) parseLine(line);
                buf.erase(0, nl + 1);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

void SerialLink::parseLine(const std::string& line) {
    if (line == "READY") {
        {
            std::lock_guard<std::mutex> lk(m_ready_mtx);
            m_ready = true;
        }
        m_ready_cv.notify_all();
        return;
    }
    if (line[0] == 'P') {
        std::istringstream ss(line);
        char tag; float p, t;
        if (ss >> tag >> p >> t) {
            std::lock_guard<std::mutex> lk(m_state_mtx);
            m_state.pan = p;
            m_state.tilt = t;
        }
    }
}

void SerialLink::send(const std::string& msg) {
    if (m_fd < 0) return;
    ssize_t n = write(m_fd, msg.data(), msg.size());
    (void)n;  // best-effort; failsafe on the ESP32 covers a dropped link
}

void SerialLink::step(float dpan, float dtilt) {
    char b[48];
    int len = snprintf(b, sizeof(b), "M %.3f %.3f\n", dpan, dtilt);
    send(std::string(b, len));
}

void SerialLink::reset() { send("R\n"); }

SerialLink::State SerialLink::getState() const {
    std::lock_guard<std::mutex> lk(m_state_mtx);
    return m_state;
}
