==============================================================
 simulation — Lập kế hoạch đường đi trên trình duyệt (Python/Flask)
==============================================================

CHỨC NĂNG
  Giao diện web để vẽ bản đồ, đặt vật cản, tìm đường (A*) và gửi goal
  xuống robot thật qua ZMQ. Hiển thị real-time pose robot trả về.
  Chạy trên laptop, kết nối WiFi tới Jetson.

CÀI ĐẶT
  pip install -r simulation/movement/requirements.txt

CHẠY
  JETSON_IP=192.168.x.x python simulation/movement/app.py
  → mở trình duyệt: http://localhost:5000

  (JETSON_IP = địa chỉ IP của Jetson trên WiFi)

ZMQ (laptop ↔ Jetson)
  5555  simulation → motivation   G <x_m> <y_m> <theta_deg>  (goal)
  5556  motivation → simulation   O <x_m> <y_m> <theta_deg>  (odom)
  Jetson BIND cả hai cổng; laptop CONNECT.
  Simulation dùng cm nội bộ, đổi sang mét trước khi gửi (CM_TO_M = 0.01).

API FLASK (cổng 5000)
  POST /api/robot/goal    {x: cm, y: cm}     → gửi goal ZMQ (mét)
  GET  /api/robot/pose    → {x_cm, y_cm, theta_deg, connected, ts}
  POST /api/robot/stop    → gửi 'S' qua ZMQ
  POST /api/pathfind      → đường A* {start, goal, obstacles}
  GET/POST /api/config    → cấu hình robot/grid/simulation

FILE CHÍNH
  movement/app.py          server Flask + cầu nối ZMQ
  movement/templates/      HTML giao diện
  movement/static/         JS/CSS
  movement/README.md       chi tiết bổ sung

GHI CHÚ
  Bên Jetson phải chạy: ./motivation --nav  (để nhận goal qua ZMQ).
