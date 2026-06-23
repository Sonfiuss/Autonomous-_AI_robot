#pragma once
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

// UART client for the ESP32 camera-servo firmware.
//
// Protocol:
//   TX  "M <dpan> <dtilt>\n"         relative step (degrees)
//   TX  "R\n"                         reset to 0°
//   RX  "P <pan> <tilt>\n"           position feedback after each move
//   RX  "READY\n"                    boot acknowledgement
class SerialLink {
public:
    struct State { float pan = 0.f; float tilt = 0.f; };

    SerialLink(std::string port, int baud = 115200);
    ~SerialLink();

    SerialLink(const SerialLink&)            = delete;
    SerialLink& operator=(const SerialLink&) = delete;

    // Open port, start RX thread, block until READY (or timeout).
    bool connect(double timeout_s = 5.0);
    void disconnect();

    // Move by a relative delta (degrees). ESP32 clamps to the limits.
    void step(float dpan, float dtilt);
    void reset();

    State getState() const;

private:
    std::string m_port;
    int         m_baud;
    int         m_fd = -1;

    mutable std::mutex m_state_mtx;
    State              m_state;

    std::mutex              m_ready_mtx;
    std::condition_variable m_ready_cv;
    bool                    m_ready = false;

    std::atomic<bool> m_running{false};
    std::thread       m_rx;

    void rxLoop();
    void parseLine(const std::string& line);
    void send(const std::string& msg);
    bool configurePort();
};
