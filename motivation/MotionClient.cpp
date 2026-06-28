#include "MotionClient.hpp"

#include <cstdio>
#include <cstring>
#include <cerrno>
#include <chrono>

double MotionClient::monoNow() {
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

bool MotionClient::configPort(int fd, int baud) {
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
    tty.c_cc[VTIME] = 1;   // 100 ms read timeout
    tcflush(fd, TCIFLUSH);
    return tcsetattr(fd, TCSANOW, &tty) == 0;
}

MotionClient::MotionClient(const std::string& port, int baud)
    : m_port(port), m_baud(baud) {}

MotionClient::~MotionClient() { disconnect(); }

bool MotionClient::connect(double timeout_s) {
    m_fd = open(m_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (m_fd < 0) {
        fprintf(stderr, "[MotionClient] Cannot open %s: %s\n",
                m_port.c_str(), strerror(errno));
        return false;
    }
    if (!configPort(m_fd, m_baud)) {
        close(m_fd); m_fd = -1; return false;
    }
    m_running = true;
    m_rx_thread = std::thread(&MotionClient::rxLoop, this);

    std::unique_lock<std::mutex> lk(m_ready_mtx);
    auto dl = std::chrono::steady_clock::now()
            + std::chrono::duration<double>(timeout_s);
    if (!m_ready_cv.wait_until(lk, dl, [this]{ return m_ready; }))
        fprintf(stderr, "[MotionClient] Timeout waiting for READY\n");

    printf("[MotionClient] Connected to %s\n", m_port.c_str());
    return true;
}

void MotionClient::disconnect() {
    m_running = false;
    if (m_rx_thread.joinable()) m_rx_thread.join();
    if (m_fd >= 0) { close(m_fd); m_fd = -1; }
}

void MotionClient::send(const std::string& msg) {
    if (m_fd < 0) return;
    if (write(m_fd, msg.c_str(), msg.size()) < 0)
        fprintf(stderr, "[MotionClient] Write error: %s\n", strerror(errno));
}

// ── Commands ──────────────────────────────────────────────────────────────────

void MotionClient::setVelocity(float vx, float vy, float omega) {
    char buf[64];
    snprintf(buf, sizeof(buf), "M %.4f %.4f %.4f\n", vx, vy, omega);
    send(buf);
}

void MotionClient::moveForward(float dist_m, float speed_ms) {
    char buf[48];
    snprintf(buf, sizeof(buf), "F %.4f %.4f\n", dist_m, speed_ms);
    send(buf);
}

void MotionClient::turn(float angle_deg, float omega_rads) {
    char buf[48];
    snprintf(buf, sizeof(buf), "T %.4f %.4f\n", angle_deg, omega_rads);
    send(buf);
}

void MotionClient::stop()          { send("S\n"); }
void MotionClient::resetOdometry() { send("R\n"); }

void MotionClient::testWheel(int idx, int revolutions) {
    char buf[48];
    int steps = revolutions * 12800;
    snprintf(buf, sizeof(buf), "W %d %d\n", idx, steps);
    send(buf);
}

void MotionClient::setServoVelocity(float pan_vel, float tilt_vel) {
    char buf[48];
    snprintf(buf, sizeof(buf), "V %.3f %.3f\n", pan_vel, tilt_vel);
    send(buf);
}

// ── State ─────────────────────────────────────────────────────────────────────

MotionClient::Odometry MotionClient::getOdometry() const {
    std::lock_guard<std::mutex> lk(m_odom_mtx);
    return m_odom;
}

MotionClient::ServoState MotionClient::getServoState() const {
    std::lock_guard<std::mutex> lk(m_servo_mtx);
    return m_servo;
}

void MotionClient::setOdomCallback(OdomCallback cb) {
    std::lock_guard<std::mutex> lk(m_cb_mtx);
    m_odom_cb = std::move(cb);
}

void MotionClient::setDoneCallback(DoneCallback cb) {
    std::lock_guard<std::mutex> lk(m_cb_mtx);
    m_done_cb = std::move(cb);
}

// ── RX loop ───────────────────────────────────────────────────────────────────

void MotionClient::rxLoop() {
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

void MotionClient::parseLine(const std::string& line) {
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
    if (line.size() > 2 && line[0] == 'O') {
        float x, y, th;
        if (sscanf(line.c_str() + 2, "%f %f %f", &x, &y, &th) == 3) {
            Odometry o{x, y, th, monoNow()};
            { std::lock_guard<std::mutex> lk(m_odom_mtx); m_odom = o; }
            OdomCallback cb;
            { std::lock_guard<std::mutex> lk(m_cb_mtx); cb = m_odom_cb; }
            if (cb) cb(o);
        }
        return;
    }
    if (line.size() > 2 && line[0] == 'P') {
        float pan, tilt;
        if (sscanf(line.c_str() + 2, "%f %f", &pan, &tilt) == 2) {
            std::lock_guard<std::mutex> lk(m_servo_mtx);
            m_servo = {pan, tilt};
        }
    }
}
