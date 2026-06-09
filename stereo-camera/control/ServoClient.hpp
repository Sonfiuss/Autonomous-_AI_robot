#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <cmath>

// POSIX serial
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

/**
 * UART client for the ESP32 servo controller.
 *
 * Protocol:
 *   TX  "V <pan_vel> <tilt_vel>\n"  – set angular velocity (deg/s)
 *   TX  "S\n"                        – stop (vel = 0)
 *   TX  "R\n"                        – reset both axes to 0°
 *   RX  "P <pan> <tilt>\n"          – position feedback at 50 Hz
 *   RX  "READY\n"                   – ESP32 boot acknowledgement
 */
class ServoClient {
public:
    struct State {
        float pan  = 0.f;  // degrees
        float tilt = 0.f;  // degrees
        double timestamp = 0.0; // seconds (CLOCK_MONOTONIC)
    };

    using PositionCallback = std::function<void(const State&)>;

    explicit ServoClient(const std::string& port, int baud = 115200);
    ~ServoClient();

    // Non-copyable
    ServoClient(const ServoClient&) = delete;
    ServoClient& operator=(const ServoClient&) = delete;

    /** Open serial port and start RX thread. Blocks until READY or timeout. */
    bool connect(double timeout_s = 5.0);

    /** Close port and stop RX thread. */
    void disconnect();

    // ── Commands ─────────────────────────────────────────────────────────────

    /** Set angular velocities (deg/s). Servo bounces at ±70° limits. */
    void setVelocity(float pan_vel, float tilt_vel);

    /** Stop both axes (set velocity to 0). */
    void stop();

    /** Move both axes back to 0°. */
    void reset();

    // ── State ─────────────────────────────────────────────────────────────────

    State getState() const;

    /**
     * Register a callback invoked on every position feedback packet.
     * Called from the RX thread – keep it fast.
     */
    void setPositionCallback(PositionCallback cb);

private:
    std::string   m_port;
    int           m_baud;
    int           m_fd = -1;

    mutable std::mutex  m_state_mutex;
    State               m_state;

    std::mutex              m_ready_mutex;
    std::condition_variable m_ready_cv;
    bool                    m_ready = false;

    std::atomic<bool>   m_running{false};
    std::thread         m_rx_thread;

    PositionCallback    m_callback;
    mutable std::mutex  m_cb_mutex;

    void rxLoop();
    void parseLine(const std::string& line);
    void send(const std::string& msg);

    static double mono_now();
    static bool   configurePort(int fd, int baud);
};
