# 6DOF Robotic Arm Simulation

Ứng dụng mô phỏng cánh tay robot 6 bậc tự do (6 Degrees of Freedom) với giao diện web tương tác.

## Tính năng

- **Mô hình 3D tương tác**: Hiển thị cánh tay robot trong không gian 3D với grid
- **Điều khiển góc khớp**: Điều chỉnh 6 góc khớp (J1-J6) bằng slider hoặc nhập trực tiếp
- **Tham số DH**: Tùy chỉnh chiều dài các link của cánh tay
- **Camera views**: Xem từ nhiều góc độ (Top, Side, Front, Free rotation)
- **Real-time update**: Cập nhật vị trí end-effector theo thời gian thực

## Cấu trúc thư mục

```
simulation/arm/
├── app.py                 # Flask backend server
├── requirements.txt       # Python dependencies
├── README.md             # Documentation
├── templates/
│   └── index.html        # Main HTML page
└── static/
    ├── css/
    │   └── style.css     # Styling
    └── js/
        ├── arm3d.js      # Three.js 3D visualization
        └── controls.js   # Control panel logic
```

## Cài đặt

1. **Tạo virtual environment** (tùy chọn):
```bash
python -m venv venv
venv\Scripts\activate  # Windows
source venv/bin/activate  # Linux/Mac
```

2. **Cài đặt dependencies**:
```bash
pip install -r requirements.txt
```

3. **Chạy server**:
```bash
python app.py
```

4. **Mở trình duyệt**:
```
http://localhost:5001
```

## Sử dụng

### Panel 3D (Bên trái)
- **Xoay camera**: Click và kéo chuột trái
- **Zoom**: Lăn chuột
- **Pan**: Click và kéo chuột phải
- **View buttons**: Chọn góc nhìn preset (Top, Side, Front, Reset)

### Panel điều khiển (Bên phải)

#### Góc các khớp (Joint Angles)
- **J1 - Base**: Xoay đế (-180° đến 180°)
- **J2 - Shoulder**: Góc vai (-90° đến 90°)
- **J3 - Elbow**: Góc khuỷu (-135° đến 135°)
- **J4 - Wrist Rotation**: Xoay cổ tay (-180° đến 180°)
- **J5 - Wrist Bend**: Uốn cổ tay (-90° đến 90°)
- **J6 - End Effector**: Xoay công cụ (-180° đến 180°)

#### Thông số DH (Link Lengths)
- **d1**: Chiều cao đế (50-200mm)
- **a2**: Khoảng cách vai-khuỷu (100-400mm)
- **a3**: Khoảng cách khuỷu-cổ tay (100-400mm)
- **d4, d5, d6**: Offset các khớp cổ tay

## API Endpoints

| Method | Endpoint | Mô tả |
|--------|----------|-------|
| GET | `/api/state` | Lấy trạng thái hiện tại |
| POST | `/api/joints` | Cập nhật góc các khớp |
| POST | `/api/dh_params` | Cập nhật thông số DH |
| POST | `/api/reset` | Reset về mặc định |

## Denavit-Hartenberg Parameters

Cánh tay sử dụng mô hình DH chuẩn với 6 khớp:

| Joint | θ | d | a | α |
|-------|---|---|---|---|
| 1 | θ1 | d1 | 0 | π/2 |
| 2 | θ2 | 0 | a2 | 0 |
| 3 | θ3 | 0 | a3 | 0 |
| 4 | θ4 | d4 | 0 | π/2 |
| 5 | θ5 | d5 | 0 | -π/2 |
| 6 | θ6 | d6 | 0 | 0 |

## Công nghệ sử dụng

- **Backend**: Flask (Python)
- **Frontend**: HTML5, CSS3, JavaScript
- **3D Rendering**: Three.js
- **Styling**: Custom CSS với dark theme

## License

MIT License - Autonomous AI Robot Project
