#include "fw/drivers/uart_link.h"

#include <cstdint>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "fw/config.h"
#include "fw/fw_debug.h"

namespace fw {

namespace {

constexpr uart_port_t PORT = static_cast<uart_port_t>(cfg::UART_PORT_NUM);

}  // namespace

UartLink::UartLink() : ready_(false) {}

bool UartLink::begin() {
    // Field by field, not a designated initialiser: those are not C++14, and
    // uart_config_t gains fields between IDF versions.
    uart_config_t config = {};
    config.baud_rate = cfg::UART_BAUD;
    config.data_bits = UART_DATA_8_BITS;
    config.parity    = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    if (uart_param_config(PORT, &config) != ESP_OK) {
        FW_DLOG("uart: param config failed\n");
        return false;
    }
    if (uart_set_pin(PORT, cfg::UART_TX_PIN, cfg::UART_RX_PIN, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE) != ESP_OK) {
        FW_DLOG("uart: set pin failed\n");
        return false;
    }
    if (uart_driver_install(PORT, cfg::UART_RX_BUFFER_BYTES, cfg::UART_TX_BUFFER_BYTES, 0, nullptr,
                            0) != ESP_OK) {
        FW_DLOG("uart: driver install failed\n");
        return false;
    }
    ready_ = true;
    return true;
}

int UartLink::read(char* out, int cap, int timeoutMs) {
    if (!ready_ || out == nullptr || cap <= 0) {
        return 0;
    }
    const int count = uart_read_bytes(PORT, reinterpret_cast<uint8_t*>(out), cap,
                                      pdMS_TO_TICKS(timeoutMs));
    return count > 0 ? count : 0;
}

bool UartLink::writeLine(const char* text, int length) {
    if (!ready_ || text == nullptr || length <= 0) {
        return false;
    }
    // Checked BEFORE writing anything: uart_write_bytes would block once the
    // ring buffer filled, and a partial write would put half a line on the
    // wire. Dropping a whole telemetry line is recoverable; a corrupt one is
    // not, because the receiver would resynchronise on the wrong boundary.
    size_t available = 0;
    if (uart_get_tx_buffer_free_size(PORT, &available) != ESP_OK) {
        return false;
    }
    if (available < static_cast<size_t>(length)) {
        return false;
    }
    return uart_write_bytes(PORT, text, static_cast<size_t>(length)) == length;
}

}  // namespace fw
