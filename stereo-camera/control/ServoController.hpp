#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

/**
 * Blocking UART client for the ESP32 servo_driver firmware.
 *
 * Protocol:
 *   TX  "MOVE <pan_f> <tilt_f>\n"  – move to position; blocks until "OK"
 *   TX  "RESET\n"                   – move to 0,0;       blocks until "OK"
 *   TX  "STOP\n"                    – hold position      (no reply)
 *   RX  "OK\n"                      – movement complete
 *   RX  "READY\n"                   – ESP32 boot acknowledgement
 */
class ServoController {
public:
    explicit ServoController(const std::string& port, int baud = 115200);
    ~ServoController();

    ServoController(const ServoController&) = delete;
    ServoController& operator=(const ServoController&) = delete;

    /** Open serial port, start RX thread. Blocks until READY or timeout. */
    bool connect(double timeout_s = 5.0);

    /** Stop RX thread and close serial port. */
    void disconnect();

    /** Send MOVE and block until OK received or timeout. */
    bool moveTo(float pan_deg, float tilt_deg, double timeout_s = 2.0);

    /** Send STOP (no reply expected). */
    void stop();

    /** Send RESET and block until OK received or timeout. */
    bool reset(double timeout_s = 2.0);

    bool isConnected() const;

private:
    std::string m_port;
    int         m_baud;
    int         m_fd = -1;

    std::mutex              m_ready_mtx;
    std::condition_variable m_ready_cv;
    bool                    m_ready = false;

    std::mutex              m_ok_mtx;
    std::condition_variable m_ok_cv;
    bool                    m_ok = false;

    std::atomic<bool> m_running{false};
    std::thread       m_rx_thread;

    bool sendAndWaitOK(const std::string& cmd, double timeout_s);
    void send(const std::string& msg);
    void rxLoop();
    void parseLine(const std::string& line);

    static bool configurePort(int fd, int baud);
};
