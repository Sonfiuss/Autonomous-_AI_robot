#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include <zmq.h>

/**
 * ZMQ bridge giữa simulation (laptop) và Jetson.
 *
 * Ports:
 *   5555  – SUB  nhận goal từ laptop:   "G <x> <y> [<theta_deg>]"
 *   5556  – PUB  gửi odometry về laptop: "O <x> <y> <theta_deg>"
 *
 * Jetson bind cả 2 port (laptop connect đến Jetson IP).
 */
class SimBridge {
public:
    struct Goal {
        float x     = 0.f;   // mét
        float y     = 0.f;
        float theta = 0.f;   // độ (tuỳ chọn – 0 nếu không có)
        bool  valid = false;
    };

    using GoalCallback = std::function<void(const Goal&)>;

    explicit SimBridge(int goal_port = 5555, int odom_port = 5556);
    ~SimBridge();

    SimBridge(const SimBridge&) = delete;
    SimBridge& operator=(const SimBridge&) = delete;

    bool start();
    void stop();

    /** Gửi odometry về simulation để hiển thị vị trí robot thực. */
    void publishOdom(float x, float y, float theta_deg);

    /** Callback được gọi mỗi khi nhận goal mới từ simulation. */
    void setGoalCallback(GoalCallback cb);

    Goal latestGoal() const;

private:
    int  m_goal_port;
    int  m_odom_port;

    void*  m_ctx     = nullptr;
    void*  m_sub_sock = nullptr;
    void*  m_pub_sock = nullptr;

    std::atomic<bool> m_running{false};
    std::thread       m_rx_thread;

    mutable std::mutex m_goal_mtx;
    Goal               m_latest_goal;

    mutable std::mutex m_cb_mtx;
    GoalCallback       m_callback;

    void rxLoop();
};
