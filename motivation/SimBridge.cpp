#include "SimBridge.hpp"
#include <cstdio>
#include <cstring>

SimBridge::SimBridge(int goal_port, int odom_port)
    : m_goal_port(goal_port), m_odom_port(odom_port) {}

SimBridge::~SimBridge() { stop(); }

bool SimBridge::start() {
    m_ctx = zmq_ctx_new();
    if (!m_ctx) return false;

    // SUB socket – nhận goal từ laptop
    m_sub_sock = zmq_socket(m_ctx, ZMQ_SUB);
    zmq_setsockopt(m_sub_sock, ZMQ_SUBSCRIBE, "", 0);   // nhận tất cả topic
    int timeout = 100;
    zmq_setsockopt(m_sub_sock, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));

    char addr[64];
    snprintf(addr, sizeof(addr), "tcp://*:%d", m_goal_port);
    if (zmq_bind(m_sub_sock, addr) != 0) {
        fprintf(stderr, "[SimBridge] Cannot bind SUB on %s\n", addr);
        return false;
    }

    // PUB socket – gửi odom về laptop
    m_pub_sock = zmq_socket(m_ctx, ZMQ_PUB);
    snprintf(addr, sizeof(addr), "tcp://*:%d", m_odom_port);
    if (zmq_bind(m_pub_sock, addr) != 0) {
        fprintf(stderr, "[SimBridge] Cannot bind PUB on %s\n", addr);
        return false;
    }

    m_running = true;
    m_rx_thread = std::thread(&SimBridge::rxLoop, this);
    printf("[SimBridge] Listening for goals on :%d, publishing odom on :%d\n",
           m_goal_port, m_odom_port);
    return true;
}

void SimBridge::stop() {
    m_running = false;
    if (m_rx_thread.joinable()) m_rx_thread.join();
    if (m_sub_sock) { zmq_close(m_sub_sock); m_sub_sock = nullptr; }
    if (m_pub_sock) { zmq_close(m_pub_sock); m_pub_sock = nullptr; }
    if (m_ctx)      { zmq_ctx_destroy(m_ctx); m_ctx = nullptr; }
}

void SimBridge::publishOdom(float x, float y, float theta_deg) {
    if (!m_pub_sock) return;
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "O %.4f %.4f %.2f", x, y, theta_deg);
    zmq_send(m_pub_sock, buf, n, ZMQ_NOBLOCK);
}

void SimBridge::setGoalCallback(GoalCallback cb) {
    std::lock_guard<std::mutex> lk(m_cb_mtx);
    m_callback = std::move(cb);
}

SimBridge::Goal SimBridge::latestGoal() const {
    std::lock_guard<std::mutex> lk(m_goal_mtx);
    return m_latest_goal;
}

void SimBridge::rxLoop() {
    char buf[128];
    while (m_running) {
        int n = zmq_recv(m_sub_sock, buf, sizeof(buf) - 1, 0);
        if (n <= 0) continue;
        buf[n] = '\0';

        // Format: "G <x> <y>" hoặc "G <x> <y> <theta_deg>"
        if (buf[0] != 'G') continue;

        Goal g;
        int parsed = sscanf(buf + 2, "%f %f %f", &g.x, &g.y, &g.theta);
        if (parsed >= 2) {
            g.valid = true;
            if (parsed < 3) g.theta = 0.f;

            { std::lock_guard<std::mutex> lk(m_goal_mtx); m_latest_goal = g; }

            GoalCallback cb;
            { std::lock_guard<std::mutex> lk(m_cb_mtx); cb = m_callback; }
            if (cb) cb(g);

            printf("[SimBridge] New goal: x=%.3f  y=%.3f  θ=%.1f°\n",
                   g.x, g.y, g.theta);
        }
    }
}
