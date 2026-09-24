#include "jetson/serial_port.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace jetson {

namespace {

constexpr int MS_PER_S  = 1000;
constexpr int US_PER_MS = 1000;

// The link runs at one speed; anything else is a caller mistake worth failing on.
bool baudConstant(int baud, speed_t* out) {
    switch (baud) {
        case 115200:
            *out = B115200;
            return true;
        case 57600:
            *out = B57600;
            return true;
        case 9600:
            *out = B9600;
            return true;
        default:
            return false;
    }
}

}  // namespace

SerialPort::SerialPort() : fd_(-1) {}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const char* device, int baud) {
    if (device == nullptr) {
        return false;
    }
    close();

    speed_t speed = B115200;
    if (!baudConstant(baud, &speed)) {
        std::fprintf(stderr, "serial: unsupported baud %d\n", baud);
        return false;
    }

    fd_ = ::open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::fprintf(stderr, "serial: cannot open %s: %s\n", device, std::strerror(errno));
        return false;
    }

    termios options;
    if (tcgetattr(fd_, &options) != 0) {
        std::fprintf(stderr, "serial: tcgetattr failed: %s\n", std::strerror(errno));
        close();
        return false;
    }

    cfmakeraw(&options);
    options.c_cflag |= CLOCAL | CREAD;      // ignore modem lines, enable the receiver
    options.c_cflag &= ~CSTOPB;             // one stop bit
    options.c_cflag &= ~PARENB;             // no parity
    options.c_cflag &= ~CRTSCTS;            // no hardware flow control
    options.c_cc[VMIN]  = 0;                // reads never block: select() does the waiting
    options.c_cc[VTIME] = 0;

    if (cfsetispeed(&options, speed) != 0 || cfsetospeed(&options, speed) != 0) {
        std::fprintf(stderr, "serial: cannot set baud: %s\n", std::strerror(errno));
        close();
        return false;
    }
    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
        std::fprintf(stderr, "serial: tcsetattr failed: %s\n", std::strerror(errno));
        close();
        return false;
    }
    tcflush(fd_, TCIOFLUSH);   // drop whatever the ESP32 said before we were listening
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

int SerialPort::read(char* out, int cap, int timeoutMs) {
    if (fd_ < 0 || out == nullptr || cap <= 0) {
        return -1;
    }

    fd_set readable;
    FD_ZERO(&readable);
    FD_SET(fd_, &readable);

    timeval timeout;
    timeout.tv_sec  = timeoutMs / MS_PER_S;
    timeout.tv_usec = (timeoutMs % MS_PER_S) * US_PER_MS;

    const int ready = select(fd_ + 1, &readable, nullptr, nullptr, &timeout);
    if (ready < 0) {
        return errno == EINTR ? 0 : -1;   // a signal is not a link failure
    }
    if (ready == 0) {
        return 0;
    }

    const ssize_t count = ::read(fd_, out, static_cast<size_t>(cap));
    if (count < 0) {
        return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
    }
    return static_cast<int>(count);
}

bool SerialPort::write(const char* text, int length) {
    if (fd_ < 0 || text == nullptr || length <= 0) {
        return false;
    }
    int written = 0;
    while (written < length) {
        const ssize_t count = ::write(fd_, text + written, static_cast<size_t>(length - written));
        if (count < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                continue;   // the kernel buffer is momentarily full
            }
            std::fprintf(stderr, "serial: write failed: %s\n", std::strerror(errno));
            return false;
        }
        written += static_cast<int>(count);
    }
    return true;
}

}  // namespace jetson
