==============================================================
 omni_wheel_module — Tài liệu & mô phỏng tham khảo bánh xe omni
==============================================================

CHỨC NĂNG
  Thư mục THAM KHẢO (không phải code chạy trên robot thật). Gồm tài liệu
  driver động cơ và một dự án mô phỏng omni 3-bánh ngoài (OpenBase) để
  tham chiếu động học và thuật toán điều khiển.

NỘI DUNG
  ARD_DRVDM556/        tài liệu/cấu hình driver DM556 (Arduino)
  OpenBase-master/     dự án mã nguồn mở OpenBase (Gazebo/ROS)
    - mô phỏng omni 3-bánh trên Gazebo
    - tài liệu suy diễn động học (forward/inverse kinematics)
    - giấy phép MIT
    - xem OpenBase-master/README.md để dùng

DÙNG ĐỂ LÀM GÌ
  - Tham chiếu công thức động học omni khi phát triển core-control / motivation
  - Mô phỏng kiểm chứng trên Gazebo trước khi chạy phần cứng thật
  - Không build/deploy trực tiếp lên Jetson

GHI CHÚ
  Động học THỰC TẾ của robot này lấy từ ESP32 firmware
  (motivation/esp32_unified_controller/), KHÔNG phải từ OpenBase.
  OpenBase chỉ dùng để đối chiếu lý thuyết.
