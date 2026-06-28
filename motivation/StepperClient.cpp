#include "StepperClient.hpp"

#include <cstdio>
#include <cstring>
#include <cerrno>
#include <chrono>

bool StepperClient::configPort(int fd, int baud) {
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) return false;
    speed_t spd = (baud == 230400) ? B230400 :
                  (baud ==  57600) ? B57600  : B115200;
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |=  CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_iflag  =  IGNBRK;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag  =  0;
    tty.c_oflag  =  0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;
    tcflush(fd, TCIFLUSH);
    return tcsetattr(fd, TCSANOW, &tty) == 0;
}

StepperClient::StepperClient(const std::string& port, int baud)
    : m_port(port), m_baud(baud) {}

StepperClient::~StepperClient() { disconnect(); }

bool StepperClient::connect(double timeout_s) {
    m_fd = open(m_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (m_fd < 0) {
        fprintf(stderr, "[StepperClient] Cannot open %s: %s\n",
                m_port.c_str(), strerror(errno));
        return false;
    }
    if (!configPort(m_fd, m_baud)) {
        close(m_fd); m_fd = -1; return false;
    }
    m_running = true;
    m_rx_thread = std::thread(&StepperClient::rxLoop, this);

    std::unique_lock<std::mutex> lk(m_ready_mtx);
    auto dl = std::chrono::steady_clock::now()
            + std::chrono::duration<double>(timeout_s);
    if (!m_ready_cv.wait_until(lk, dl, [this]{ return m_ready; }))
        fprintf(stderr, "[StepperClient] Timeout waiting for READY\n");

    printf("[StepperClient] Connected to %s\n", m_port.c_str());
    return true;
}

void StepperClient::disconnect() {
    m_running = false;
    if (m_rx_thread.joinable()) m_rx_thread.join();
    if (m_fd >= 0) { close(m_fd); m_fd = -1; }
}

void StepperClient::send(const std::string& msg) {
    if (m_fd < 0) return;
    if (write(m_fd, msg.c_str(), msg.size()) < 0)
        fprintf(stderr, "[StepperClient] Write error: %s\n", strerror(errno));
}

// ── Commands ──────────────────────────────────────────────────────────────────

void StepperClient::spin(int idx, float hz) {
    char buf[32];
    snprintf(buf, sizeof(buf), "C %d %.1f\n", idx, hz);
    send(buf);
}

void StepperClient::moveSteps(int idx, int steps, uint32_t hz) {
    char buf[48];
    snprintf(buf, sizeof(buf), "W %d %d %u\n", idx, steps, hz);
    send(buf);
}

void StepperClient::stopMotor(int idx) {
    char buf[16];
    snprintf(buf, sizeof(buf), "C %d 0\n", idx);
    send(buf);
}

void StepperClient::stopAll() { send("S\n"); }

void StepperClient::resetPosition() { send("R\n"); }

void StepperClient::setDoneCallback(DoneCallback cb) {
    std::lock_guard<std::mutex> lk(m_cb_mtx);
    m_done_cb = std::move(cb);
}

// ── RX loop ───────────────────────────────────────────────────────────────────

void StepperClient::rxLoop() {
    std::string buf;
    char ch;
    while (m_running) {
        ssize_t n = read(m_fd, &ch, 1);
        if (n <= 0) continue;
        if (ch == '\n') {
            if (!buf.empty() && buf.back() == '\r') buf.pop_back();
            if (!buf.empty()) parseLine(buf);
            buf.clear();
        } else {
            buf += ch;
        }
    }
}

void StepperClient::parseLine(const std::string& line) {
    if (line == "READY") {
        { std::lock_guard<std::mutex> lk(m_ready_mtx); m_ready = true; }
        m_ready_cv.notify_all();
        return;
    }
    if (line == "K") {
        DoneCallback cb;
        { std::lock_guard<std::mutex> lk(m_cb_mtx); cb = m_done_cb; }
        if (cb) cb();
        return;
    }
    // O / P lines are silently ignored
}
