/**
 * @file    fan_config.h
 * @brief   Cấu hình pin và tham số cho Fan Controller - Arduino Uno
 *
 * Quạt: Prism 6PRO  –  120×120×25mm  –  DC 12V  –  2 dây dùng
 * Kết nối:
 *   Dây ĐỎ  (12V) ──── Nguồn 12V DC ngoài
 *   Dây ĐEN (GND) ──── Drain MOSFET → Source → GND chung
 *
 * Điều tốc: N-Channel MOSFET (IRLZ44N / IRFZ44N)
 *   Gate  ──[100Ω]── Arduino Pin 5 (PWM)
 *   Drain ────────── Dây GND quạt (đen)
 *   Source ───────── GND chung
 */

#ifndef FAN_CONFIG_H
#define FAN_CONFIG_H

/* ================================================================
   CẤU HÌNH PIN
   ================================================================ */

/** Pin PWM → Gate MOSFET (qua điện trở 100Ω)
 *  PWM capable trên Uno: pin 3, 5, 6, 9, 10, 11
 */
#define PIN_PWM         5

/**
 * PWM tối thiểu để MOSFET dẫn đủ điện áp cho quạt bắt đầu quay
 * Quạt brushless 3-pin có IC driver bên trong: duty quá thấp khiến
 * driver reset liên tục → quạt jitter, không quay được.
 * 180/255 ≈ 70% – đủ để driver IC giữ nguồn và khởi động.
 * Sau khi quạt đã quay ổn định có thể giảm dần qua lệnh SPD#<n>.
 */
#define MIN_START_PWM           255  // 95% of 255

/* ================================================================
   CẤU HÌNH SERIAL
   ================================================================ */
#define SERIAL_BAUD             9600

/* ================================================================
   TÍNH NĂNG TÙY CHỌN
   ================================================================ */

/** 1 = Dùng biến trở điều chỉnh tốc độ | 0 = Chỉ dùng Serial */
#define ENABLE_POT_CONTROL      0

#endif /* FAN_CONFIG_H */
