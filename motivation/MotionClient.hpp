#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <functional>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

/**
 * Jetson-side UART client cho ESP32 Unified Controller.
 *
 * Protocol TX:
 *   M <vx> <vy> <omega>\n   – velocity liên tục (m/s, m/s, rad/s)
 *   F <dist_m> <spd_ms>\n   – di chuyển thẳng
 *   T <angle_deg> <rads>\n  – xoay
 *   V <pan_v> <tilt_v>\n    – servo camera (deg/s)
 *   S\n                     – dừng tất cả
 *   R\n                     – reset odometry
 *
 * Protocol RX:
 *   O <x> <y> <theta_deg>\n – odometry (10 Hz)
 *   P <pan> <tilt>\n        – servo angles (50 Hz)
 *   K\n                     – motion done
 *   READY\n                 – boot ack
 */
class MotionClient {
public:
    struct Odometry {
        float x     = 0.f;    // m
        float y     = 0.f;    // m
        float theta = 0.f;    // degrees
        double timestamp = 0.0;
    };

    struct ServoState {
        float pan  = 0.f;
        float tilt = 0.f;
    };

    using OdomCallback  = std::function<void(const Odometry&)>;
    using DoneCallback  = std::function<void()>;

    explicit MotionClient(const std::string& port, int baud = 115200);
    ~MotionClient();

    MotionClient(const MotionClient&) = delete;
    MotionClient& operator=(const MotionClient&) = delete;

    bool connect(double timeout_s = 5.0);
    void disconnect();

    // ── Motion commands ───────────────────────────────────────────────────────
    void setVelocity(float vx, float vy, float omega);
    void moveForward(float dist_m, float speed_ms);
    void turn(float angle_deg, float omega_rads);
    void stop();
    void resetOdometry();

    // ── Servo commands ────────────────────────────────────────────────────────
    void setServoVelocity(float pan_vel, float tilt_vel);

    // ── State ─────────────────────────────────────────────────────────────────
    Odometry   getOdometry()   const;
    ServoState getServoState() const;

    void setOdomCallback(OdomCallback cb);
    void setDoneCallback(DoneCallback cb);

private:
    std::string m_port;
    int         m_baud;
    int         m_fd = -1;

    mutable std::mutex m_odom_mtx;
    Odometry           m_odom;

    mutable std::mutex m_servo_mtx;
    ServoState         m_servo;

    std::mutex              m_ready_mtx;
    std::condition_variable m_ready_cv;
    bool                    m_ready = false;

    std::atomic<bool> m_running{false};
    std::thread       m_rx_thread;

    mutable std::mutex m_cb_mtx;
    OdomCallback       m_odom_cb;
    DoneCallback       m_done_cb;

    void rxLoop();
    void parseLine(const std::string& line);
    void send(const std::string& msg);

    static double monoNow();
    static bool   configPort(int fd, int baud);
};
