// Uart_Link — the one place the firmware touches the serial port. The Comm task
// reads from it and the Status task writes to it; nothing else opens it, which
// is what keeps telemetry lines from interleaving.
#ifndef FW_UART_LINK_H
#define FW_UART_LINK_H

namespace fw {

class UartLink {
public:
    UartLink();

    // Installs the UART driver. False if any step failed; the caller must not
    // start the tasks in that case.
    bool begin();

    // Reads up to `cap` bytes, waiting at most `timeoutMs`. Returns the number
    // read, or 0 for "nothing arrived" — errors are reported as 0 too, since a
    // reader has nothing useful to do about them but try again.
    int read(char* out, int cap, int timeoutMs);

    // Writes one whole line, or none of it. Returns false when the TX buffer
    // cannot take the entire line: the caller counts a dropped message instead
    // of blocking, and a half-written line never reaches the wire.
    bool writeLine(const char* text, int length);

private:
    bool ready_;
};

}  // namespace fw

#endif  // FW_UART_LINK_H
