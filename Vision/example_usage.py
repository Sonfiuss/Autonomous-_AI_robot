"""Ví dụ sử dụng ImouCamera (đọc config từ camera_config.json)."""

import os
import time

from imou_camera_capture import ImouCamera, load_camera_config


def _make_camera() -> ImouCamera:
    base_dir = os.path.dirname(__file__)
    cfg_path = os.path.join(base_dir, "camera_config.json")
    cfg = load_camera_config(cfg_path)
    return ImouCamera(cfg)

def example_1_simple_capture():
    """Ví dụ 1: Chụp một ảnh đơn giản"""
    print("\n=== VÍ DỤ 1: CHỤP MỘT ẢNH ===\n")

    camera = _make_camera()
    
    if camera.connect():
        frame = camera.capture_frame()
        if frame is not None:
            path = camera.save_image(frame)
            print(f"✓ Đã lưu: {path}")
        camera.disconnect()


def example_2_timelapse():
    """Ví dụ 2: Chụp timelapse - chụp ảnh định kỳ"""
    print("\n=== VÍ DỤ 2: CHỤP TIMELAPSE ===\n")
    
    camera = _make_camera()
    
    if camera.connect():
        # Chụp 20 ảnh, mỗi ảnh cách nhau 5 giây
        paths = camera.capture_multiple_images(count=20, interval=5)
        print(f"✓ Đã lưu {len(paths)} ảnh")
        camera.disconnect()


def example_3_continuous_monitoring():
    """Ví dụ 3: Giám sát liên tục và lưu ảnh mỗi phút"""
    print("\n=== VÍ DỤ 3: GIÁM SÁT LIÊN TỤC ===\n")
    
    camera = _make_camera()
    
    if camera.connect():
        print("Bắt đầu giám sát. Nhấn Ctrl+C để dừng.")
        try:
            while True:
                frame = camera.capture_frame()
                if frame is not None:
                    path = camera.save_image(frame, folder="monitoring")
                    print(f"✓ Đã lưu: {path}")
                    print("Ảnh tiếp theo sau 60 giây...")
                time.sleep(60)  # Chụp mỗi phút
        except KeyboardInterrupt:
            print("\nĐã dừng giám sát")
        finally:
            camera.disconnect()


def example_4_multiple_cameras():
    """Ví dụ 4: Kết nối với nhiều camera"""
    print("\n=== VÍ DỤ 4: NHIỀU CAMERA ===\n")

    # Gợi ý: Nếu bạn có nhiều camera, hãy tạo nhiều file config riêng
    # (ví dụ: camera_config_1.json, camera_config_2.json, ...) rồi load từng file.
    base_dir = os.path.dirname(__file__)
    config_paths = [
        os.path.join(base_dir, "camera_config.json"),
        # os.path.join(base_dir, "camera_config_2.json"),
        # os.path.join(base_dir, "camera_config_3.json"),
    ]
    cameras = [ImouCamera(load_camera_config(p)) for p in config_paths]
    
    connected_cameras = []
    
    # Kết nối tất cả camera
    for i, camera in enumerate(cameras):
        print(f"\nKết nối camera {i+1}...")
        if camera.connect():
            connected_cameras.append((i+1, camera))
    
    # Chụp ảnh từ tất cả camera
    print(f"\n✓ Đã kết nối {len(connected_cameras)} camera")
    for idx, camera in connected_cameras:
        frame = camera.capture_frame()
        if frame is not None:
            camera.save_image(frame, folder=f"camera_{idx}")
    
    # Ngắt kết nối tất cả
    for _, camera in connected_cameras:
        camera.disconnect()


def example_5_save_with_custom_name():
    """Ví dụ 5: Lưu ảnh với tên tùy chỉnh"""
    print("\n=== VÍ DỤ 5: TÊN TÙY CHỈNH ===\n")
    
    import cv2
    from datetime import datetime
    
    camera = _make_camera()
    
    if camera.connect():
        frame = camera.capture_frame()
        if frame is not None:
            # Lưu với tên tùy chỉnh
            filename = f"my_custom_image_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
            cv2.imwrite(filename, frame)
            print(f"✓ Đã lưu: {filename}")
        camera.disconnect()


def example_6_check_camera_info():
    """Ví dụ 6: Lấy thông tin camera"""
    print("\n=== VÍ DỤ 6: THÔNG TIN CAMERA ===\n")
    
    import cv2
    
    camera = _make_camera()
    
    if camera.connect():
        frame = camera.capture_frame()
        if frame is not None:
            height, width, channels = frame.shape
            print(f"Độ phân giải: {width}x{height}")
            print(f"Số kênh màu: {channels}")
            print(f"FPS: {camera.cap.get(cv2.CAP_PROP_FPS)}")
        camera.disconnect()


if __name__ == "__main__":
    print("CHƯƠNG TRÌNH VÍ DỤ SỬ DỤNG IMOU CAMERA")
    print("="*60)
    print("\nLưu ý: Thay đổi IP và mật khẩu camera trước khi chạy!")
    print("\nCác ví dụ có sẵn:")
    print("1. Chụp một ảnh đơn giản")
    print("2. Chụp timelapse")
    print("3. Giám sát liên tục")
    print("4. Nhiều camera")
    print("5. Lưu với tên tùy chỉnh")
    print("6. Kiểm tra thông tin camera")
    
    choice = input("\nChọn ví dụ muốn chạy (1-6): ").strip()
    
    if choice == "1":
        example_1_simple_capture()
    elif choice == "2":
        example_2_timelapse()
    elif choice == "3":
        example_3_continuous_monitoring()
    elif choice == "4":
        example_4_multiple_cameras()
    elif choice == "5":
        example_5_save_with_custom_name()
    elif choice == "6":
        example_6_check_camera_info()
    else:
        print("Lựa chọn không hợp lệ!")
