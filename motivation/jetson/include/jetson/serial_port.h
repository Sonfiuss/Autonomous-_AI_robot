// Serial_Port — POSIX termios wrapper around /dev/ttyUSB0.
//
// Linux only: this is the Jetson half of the link and does not build on
// Windows. Non-copyable, because two objects sharing one descriptor would
// close it twice.
#ifndef JETSON_SERIAL_PORT_H
#define JETSON_SERIAL_PORT_H

namespace jetson {

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    SerialPort(const SerialPort&)            = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Opens the device raw at `baud` (8N1, no flow control, no echo). Returns
    // false on any failure, having closed the descriptor again first.
    bool open(const char* device, int baud);

    void close();
    bool isOpen() const { return fd_ >= 0; }

    // Reads whatever has arrived, waiting at most `timeoutMs`. Returns the byte
    // count, 0 on timeout, -1 on error.
    int read(char* out, int cap, int timeoutMs);

    // Writes the whole buffer, resuming after a partial write. False on error.
    bool write(const char* text, int length);

private:
    int fd_;
};

}  // namespace jetson

#endif  // JETSON_SERIAL_PORT_H
