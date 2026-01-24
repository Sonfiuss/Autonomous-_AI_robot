# Imou Camera Capture - Chương trình nhận hình ảnh từ Camera Imou

## Giới thiệu
Chương trình Python cho phép kết nối và nhận hình ảnh từ camera Imou trong cùng mạng LAN thông qua giao thức RTSP.

## Tính năng
- ✅ Kết nối với camera Imou qua RTSP
- ✅ Xem video trực tiếp từ camera
- ✅ Chụp ảnh và lưu vào máy tính
- ✅ Chụp nhiều ảnh liên tiếp
- ✅ Hiển thị thông tin camera trên video

## Yêu cầu hệ thống
- Python 3.7 trở lên
- Camera Imou trong cùng mạng LAN
- OpenCV

## Cài đặt

### 1. Cài đặt các thư viện cần thiết
```bash
pip install -r requirements.txt
```

### 2. Cấu hình camera

Trước khi chạy chương trình, bạn cần:

#### a. Tìm địa chỉ IP của camera
- Mở ứng dụng Imou Life trên điện thoại
- Vào Settings → Device Information → IP Address
- Hoặc vào router để xem danh sách thiết bị

#### b. Bật RTSP trên camera (nếu chưa bật)
- Mở ứng dụng Imou Life
- Chọn camera → Settings
- Tìm mục "RTSP" và bật lên
- Ghi nhớ username và password (thường là admin/admin hoặc admin/password)

#### c. Chỉnh sửa file `imou_camera_capture.py`
Tìm và thay đổi các dòng sau trong hàm `main()`:
```python
CAMERA_IP = "192.168.1.100"  # Thay bằng IP camera của bạn
USERNAME = "admin"            # Tên đăng nhập
PASSWORD = "your_password"    # Mật khẩu camera
```

## Sử dụng

### Chạy chương trình
```bash
python imou_camera_capture.py
```

### Menu chức năng
1. **Xem video trực tiếp**: Hiển thị video từ camera real-time
   - Nhấn `q`: Thoát
   - Nhấn `s`: Chụp và lưu ảnh
   - Nhấn `SPACE`: Tạm dừng/tiếp tục

2. **Chụp 1 ảnh**: Chụp và lưu một ảnh ngay lập tức

3. **Chụp nhiều ảnh liên tiếp**: Chụp nhiều ảnh với khoảng thời gian tùy chỉnh

4. **Thoát**: Đóng chương trình

### Ảnh được lưu ở đâu?
Ảnh được lưu trong thư mục `captured_images/` với tên file dạng:
```
imou_capture_YYYYMMDD_HHMMSS.jpg
```

## Xử lý lỗi

### Lỗi: "Không thể kết nối với camera"
**Nguyên nhân và cách khắc phục:**

1. **IP camera không đúng**
   - Kiểm tra lại IP trong ứng dụng Imou Life
   - Thử ping camera: `ping 192.168.1.100`

2. **Camera và máy tính không cùng mạng**
   - Đảm bảo cả hai đều kết nối vào cùng WiFi/Router

3. **Username/Password sai**
   - Kiểm tra lại thông tin đăng nhập
   - Thử reset password camera

4. **RTSP chưa được bật**
   - Vào app Imou Life → Settings → RTSP → Enable

5. **Firewall chặn**
   - Tắt tạm thời firewall để kiểm tra
   - Mở port 554 (RTSP) trên firewall

### Lỗi: "Không thể đọc frame từ camera"
- Kiểm tra kết nối mạng
- Khởi động lại camera
- Thử giảm chất lượng stream (subtype=1 thay vì 0)

## Các URL RTSP thường dùng cho Imou

Chương trình tự động thử các định dạng sau:
```
rtsp://username:password@ip:554/cam/realmonitor?channel=1&subtype=0
rtsp://username:password@ip:554/stream1
rtsp://username:password@ip:554/stream2
rtsp://username:password@ip:554/live
rtsp://username:password@ip:554/Streaming/Channels/101
rtsp://username:password@ip:554/11
```

## Tùy chỉnh

### Thay đổi chất lượng video
Trong URL RTSP, thay đổi `subtype`:
- `subtype=0`: Main stream (chất lượng cao)
- `subtype=1`: Sub stream (chất lượng thấp, ít băng thông)

### Thay đổi kênh camera
Nếu camera có nhiều kênh, thay đổi `channel`:
- `channel=1`: Kênh 1
- `channel=2`: Kênh 2

## Ví dụ sử dụng trong code

```python
from imou_camera_capture import ImouCamera

# Khởi tạo camera
camera = ImouCamera("192.168.1.100", "admin", "password")

# Kết nối
if camera.connect():
    # Chụp một ảnh
    frame = camera.capture_frame()
    camera.save_image(frame)
    
    # Ngắt kết nối
    camera.disconnect()
```

## Hỗ trợ
Nếu gặp vấn đề, hãy kiểm tra:
- Camera có sáng đèn LED không?
- Camera có phản hồi ping không?
- RTSP có được bật trong cài đặt camera không?
- Thử truy cập camera qua ứng dụng Imou Life trước

## License
MIT License

## Tác giả
Created for Autonomous AI Robot Vision System
