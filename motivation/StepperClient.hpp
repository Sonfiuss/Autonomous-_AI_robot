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
 * Jetson-side UART client — direct per-motor stepper control.
 *
 * Protocol TX (→ ESP32):
 *   C <idx> <hz>\n   continuous spin: idx=0-2, hz>0 forward, hz<0 backward, 0 stop
 *   W <idx> <steps> <hz>\n   move N steps then stop (sends K when done)
 *   S\n              stop all motors
 *   R\n              reset all step counters to 0
 *
 * Protocol RX (← ESP32):
 *   K\n              motion-complete ack
 *   O <x> <y> <th>\n odometry at 10 Hz (ignored here)
 *   READY\n          boot ack
 *
 * Motor index:  0 = W1 (60°)   1 = W2 (180°)   2 = W3 (300°)
 */
class StepperClient {
public:
    using DoneCallback = std::function<void()>;

    explicit StepperClient(const std::string& port, int baud = 115200);
    ~StepperClient();

    StepperClient(const StepperClient&) = delete;
    StepperClient& operator=(const StepperClient&) = delete;

    bool connect(double timeout_s = 5.0);
    void disconnect();

    // Spin motor idx continuously. hz > 0 = forward, hz < 0 = backward, 0 = stop.
    void spin(int idx, float hz);

    // Move motor idx by N steps at given hz, then stop. K is sent when done.
    void moveSteps(int idx, int steps, uint32_t hz = 2000);

    // Stop one motor (equivalent to spin(idx, 0)).
    void stopMotor(int idx);

    // Stop all three motors.
    void stopAll();

    // Reset step counters to 0 on the ESP32.
    void resetPosition();

    // Optional callback fired when ESP32 sends K (motion complete).
    void setDoneCallback(DoneCallback cb);

private:
    std::string m_port;
    int         m_baud;
    int         m_fd = -1;

    std::mutex              m_ready_mtx;
    std::condition_variable m_ready_cv;
    bool                    m_ready = false;

    std::atomic<bool> m_running{false};
    std::thread       m_rx_thread;

    std::mutex   m_cb_mtx;
    DoneCallback m_done_cb;

    void rxLoop();
    void parseLine(const std::string& line);
    void send(const std::string& msg);

    static bool configPort(int fd, int baud);
};
