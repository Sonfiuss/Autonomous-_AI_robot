# Fan Controller – Arduino Uno (Quạt 3 Chân DC 12V)

Firmware điều khiển **quạt 3 chân DC 12V** qua N-Channel MOSFET.  
Model thử nghiệm: **Prism 6PRO** — 120×120×25mm — 765~1800 RPM.

---

## Sơ đồ kết nối

```
  Nguồn 12V DC ngoài
       │
      [+]──────────────────────────── Dây ĐỎ  (12V quạt)
      [-]──── GND chung ──────────── Dây ĐEN* (GND quạt qua MOSFET)
                                              │
                                         D (Drain)
                                       ┌──────────┐
  Arduino Pin 5 ──[100Ω]──── Gate  │  IRLZ44N │  (N-CH MOSFET)
                                       └──────────┘
                                         S (Source)
                                              │
                                         GND chung

  Dây VÀNG (TACH) ──[10kΩ pull-up → 5V]── Arduino Pin 2 (INT0)

  Biến trở 10kΩ:  5V ── [pot] ── GND
                              └── Arduino A0  (tùy chọn)

  Tụ lọc 100µF/25V song song trên đường 12V
```

> `*` Dây **ĐEN** của quạt **không** nối thẳng GND — nối vào **Drain** MOSFET.  
> Source MOSFET mới nối GND chung cùng Arduino và nguồn 12V.

---

## Chi tiết 3 chân quạt

| Dây   | Màu  | Kết nối                              |
|:-----:|:----:|--------------------------------------|
| 12V   | Đỏ   | Nguồn 12V DC ngoài                   |
| GND   | Đen  | Drain MOSFET (Source → GND chung)    |
| TACH  | Vàng | Pin 2 qua 10kΩ pull-up lên 5V        |

---

## Cấu trúc file

```
fan/
├── fan_control.ino   # Main sketch – loop, ISR, Serial handler
└── fan_config.h      # Định nghĩa pin, tham số Prism 6PRO
```

---

## Lệnh Serial (9600 baud)

| Lệnh | Mô tả                        |
|:----:|------------------------------|
| `R`  | **Chạy** quạt (tốc độ mặc định ~50%) |
| `S`  | **Dừng** quạt                |
| `1`–`9` | Tốc độ **10% → 90%**    |
| `+`  | Tốc độ **100%**              |
| `0`  | Tốc độ **0%** (dừng mềm)    |

Output Serial mỗi 500 ms:
```
[STATUS] CHAY  PWM=180 (70%)  RPM=1260
```

---

## Cấu hình (`fan_config.h`)

| Macro                 | Mặc định | Mô tả                                    |
|-----------------------|:--------:|------------------------------------------|
| `PIN_PWM`             | `5`      | PWM → Gate MOSFET                        |
| `PIN_TACH`            | `2`      | TACH vào INT0                            |
| `PIN_POT`             | `A0`     | Biến trở (tùy chọn)                      |
| `TACH_PULSES_PER_REV` | `2`      | Xung/vòng (Prism 6PRO = 2)               |
| `RPM_INTERVAL_MS`     | `500`    | Chu kỳ cập nhật RPM                      |
| `DEFAULT_SPEED_PWM`   | `128`    | Tốc độ mặc định khi bật (~50%)           |
| `MIN_START_PWM`       | `80`     | PWM tối thiểu để quạt khởi động          |
| `ENABLE_POT_CONTROL`  | `0`      | `1` = dùng biến trở, `0` = chỉ Serial   |

---

## Yêu cầu phần cứng

| Linh kiện | Ghi chú |
|-----------|---------|
| Arduino Uno R3 | – |
| N-CH MOSFET **IRLZ44N** | Logic-level gate, khuyến nghị |
| Điện trở **100Ω** | Hạn dòng gate |
| Điện trở **10kΩ** | Pull-up dây TACH lên 5V |
| Tụ **100µF / 25V** | Lọc nguồn 12V |
| Nguồn **DC 12V ≥ 0.5A** | Riêng cho quạt |
| Biến trở 10kΩ | Tùy chọn |

