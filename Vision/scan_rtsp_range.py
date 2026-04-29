"""Quét dải IP 192.168.1.x để tìm camera Imou.

Thử kết nối RTSP với user=admin, password=L2B274DC, port=554.
Nếu tìm được camera, tự động ghi IP vào camera_config.json.

Cách chạy:
    python scan_rtsp_range.py
"""

import json
import socket
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import cv2

# ============ CẤU HÌNH CỐ ĐỊNH ============
SUBNET = "192.168.1"
START_HOST = 1
END_HOST = 254
USERNAME = "admin"
PASSWORD = "L2B274DC"
RTSP_PORT = 554
TIMEOUT_S = 2.0  # timeout cho mỗi lần thử kết nối OpenCV
WORKERS = 32     # số luồng song song (giảm nếu CPU yếu)

# Các URL pattern phổ biến của camera Imou/Dahua
RTSP_PATHS = [
    "/cam/realmonitor?channel=1&subtype=0",
    "/cam/realmonitor?channel=1&subtype=1",
    "/stream1",
    "/live",
    "/11",
    "/Streaming/Channels/101",
]


def try_rtsp_connect(ip: str) -> bool:
    """Thử kết nối RTSP tới IP với các URL pattern khác nhau. Trả về True nếu thành công."""
    for path in RTSP_PATHS:
        url = f"rtsp://{USERNAME}:{PASSWORD}@{ip}:{RTSP_PORT}{path}"
        cap = cv2.VideoCapture(url)
        
        # Đợi tối đa TIMEOUT_S giây
        start = time.time()
        while time.time() - start < TIMEOUT_S:
            if cap.isOpened():
                # Thử đọc 1 frame để chắc chắn
                ret, _ = cap.read()
                cap.release()
                if ret:
                    return True
                break
            time.sleep(0.1)
        
        cap.release()
    
    return False


def check_port_open(ip: str, port: int, timeout: float = 0.3) -> bool:
    """Kiểm tra nhanh xem port có mở không (TCP connect)."""
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except (socket.timeout, ConnectionRefusedError, OSError):
        return False


def scan_single_ip(ip: str) -> tuple[str, bool]:
    """Quét 1 IP: kiểm tra port 554 mở, nếu có thì thử kết nối RTSP."""
    # Bước 1: Kiểm tra nhanh port 554
    if not check_port_open(ip, RTSP_PORT, timeout=0.3):
        return ip, False
    
    # Bước 2: Thử kết nối RTSP thật
    print(f"  Port 554 open at {ip}, trying RTSP connection...")
    success = try_rtsp_connect(ip)
    return ip, success


def update_camera_config(ip: str, config_path: Path) -> None:
    """Cập nhật ip_address trong camera_config.json."""
    if config_path.exists():
        data = json.loads(config_path.read_text(encoding="utf-8"))
    else:
        data = {}
    
    data["ip_address"] = ip
    data.setdefault("username", USERNAME)
    data.setdefault("password", PASSWORD)
    data.setdefault("rtsp_port", RTSP_PORT)
    
    config_path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def main():
    print("=" * 60)
    print("QUÉT CAMERA IMOU TRONG DẢI 192.168.1.x")
    print("=" * 60)
    print(f"User: {USERNAME}")
    print(f"Password: {PASSWORD}")
    print(f"Port: {RTSP_PORT}")
    print(f"Dải quét: {SUBNET}.{START_HOST} -> {SUBNET}.{END_HOST}")
    print("-" * 60)
    
    hosts = [f"{SUBNET}.{i}" for i in range(START_HOST, END_HOST + 1)]
    found_cameras: list[str] = []
    
    print("Đang quét... (có thể mất 1-2 phút)")
    
    with ThreadPoolExecutor(max_workers=WORKERS) as executor:
        futures = {executor.submit(scan_single_ip, ip): ip for ip in hosts}
        done_count = 0
        total = len(futures)
        
        for future in as_completed(futures):
            done_count += 1
            ip, success = future.result()
            
            if success:
                print(f"  ✓ TÌM THẤY CAMERA: {ip}")
                found_cameras.append(ip)
            
            # Hiện progress mỗi 50 IP
            if done_count % 50 == 0 or done_count == total:
                print(f"  Progress: {done_count}/{total}")
    
    print("-" * 60)
    
    if not found_cameras:
        print("Không tìm thấy camera nào!")
        print("\nGợi ý:")
        print("  1. Kiểm tra camera đã bật và kết nối WiFi/LAN chưa")
        print("  2. Kiểm tra máy tính có cùng mạng với camera không (192.168.1.x)")
        print("  3. Kiểm tra user/password có đúng không")
        print("  4. Thử bật RTSP trong app Imou Life")
        return
    
    print(f"\n✓ Tìm thấy {len(found_cameras)} camera:")
    for ip in found_cameras:
        print(f"    - {ip}")
    
    # Ghi vào camera_config.json
    base_dir = Path(__file__).resolve().parent
    config_path = base_dir / "camera_config.json"
    
    if len(found_cameras) == 1:
        camera_ip = found_cameras[0]
        update_camera_config(camera_ip, config_path)
        print(f"\n✓ Đã cập nhật {config_path.name}: ip_address = {camera_ip}")
    else:
        print(f"\nCó nhiều camera, vui lòng chọn IP và sửa thủ công trong {config_path.name}")
        # Ghi IP đầu tiên
        camera_ip = found_cameras[0]
        update_camera_config(camera_ip, config_path)
        print(f"(Tạm thời đã ghi IP đầu tiên: {camera_ip})")


if __name__ == "__main__":
    main()
