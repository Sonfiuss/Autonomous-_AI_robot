#!/usr/bin/env python3
"""
Servo Controller - Giao tiếp giữa Hand Angle Detector và Arduino
Ánh xạ góc bàn tay -> góc servo với smoothing và safety checks
"""

import serial
import time
import threading
import queue

class ServoController:
    def __init__(self, port='COM3', baudrate=115200, num_servos=6):
        """
        Khởi tạo kết nối Serial với Arduino
        
        Args:
            port: Cổng COM (VD: 'COM3', '/dev/ttyUSB0')
            baudrate: Tốc độ truyền (115200)
            num_servos: Số lượng servo (6)
        """
        self.port = port
        self.baudrate = baudrate
        self.num_servos = num_servos
        self.connected = False
        
        # Command queue cho thread-safe operation
        self.command_queue = queue.Queue(maxsize=10)
        
        # Current servo angles
        self.current_angles = [90.0] * num_servos
        
        # Thread control
        self.running = False
        self.send_thread = None
        
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            time.sleep(2)  # Đợi Arduino reset
            self.connected = True
            print(f"✓ Connected to {port} at {baudrate} baud")
            
            # Đọc welcome message
            time.sleep(0.1)
            while self.ser.in_waiting:
                msg = self.ser.readline().decode().strip()
                print(f"  Arduino: {msg}")
                
            # Start sender thread
            self.running = True
            self.send_thread = threading.Thread(target=self._sender_loop, daemon=True)
            self.send_thread.start()
            
        except serial.SerialException as e:
            print(f"✗ Failed to connect to {port}: {e}")
            self.connected = False
    
    def _sender_loop(self):
        """Background thread gửi lệnh từ queue"""
        while self.running:
            try:
                # Lấy command từ queue (blocking với timeout)
                angles = self.command_queue.get(timeout=0.05)
                self._send_command_direct(angles)
                self.command_queue.task_done()
            except queue.Empty:
                pass
            except Exception as e:
                print(f"Error in sender loop: {e}")
    
    def _send_command_direct(self, angles):
        """Gửi lệnh trực tiếp qua Serial (internal use)"""
        if not self.connected:
            return False
        
        try:
            # Format: <a0,a1,a2,a3,a4,a5>\n
            cmd = '<' + ','.join([f"{a:.1f}" for a in angles]) + '>\n'
            self.ser.write(cmd.encode())
            self.current_angles = list(angles)
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False
    
    def send_angles(self, angles, blocking=False):
        """
        Gửi góc servo (non-blocking qua queue)
        
        Args:
            angles: List[float] - 6 giá trị góc (0-180)
            blocking: bool - Đợi lệnh được gửi xong (default: False)
        """
        if not self.connected:
            return False
        
        if len(angles) != self.num_servos:
            print(f"Error: Expected {self.num_servos} angles, got {len(angles)}")
            return False
        
        # Clamp angles to valid range
        clamped = [max(0, min(180, a)) for a in angles]
        
        try:
            # Thêm vào queue (non-blocking)
            self.command_queue.put_nowait(clamped)
            
            if blocking:
                self.command_queue.join()
            
            return True
        except queue.Full:
            # Queue đầy, bỏ qua lệnh cũ (keep latest)
            return False
    
    def map_hand_angle_to_servo(self, hand_angle, servo_index=0, 
                                 min_angle=0, max_angle=180,
                                 invert=False):
        """
        Ánh xạ góc bàn tay (0-180°) sang góc servo cụ thể
        
        Args:
            hand_angle: Góc đo được từ bàn tay (0-180°)
            servo_index: Index của servo cần điều khiển (0-5)
            min_angle: Góc servo min khi hand_angle = 0
            max_angle: Góc servo max khi hand_angle = 180
            invert: Đảo chiều ánh xạ
            
        Returns:
            float: Góc servo tương ứng
        """
        if invert:
            # Hand angle lớn -> servo angle nhỏ
            ratio = 1.0 - (hand_angle / 180.0)
        else:
            # Hand angle lớn -> servo angle lớn
            ratio = hand_angle / 180.0
        
        servo_angle = min_angle + ratio * (max_angle - min_angle)
        return max(0, min(180, servo_angle))
    
    def control_single_servo(self, servo_index, hand_angle, 
                            min_angle=0, max_angle=180, invert=False):
        """
        Điều khiển 1 servo dựa trên góc bàn tay, giữ các servo khác nguyên
        
        Args:
            servo_index: Index servo cần điều khiển (0-5)
            hand_angle: Góc từ bàn tay (0-180)
            min_angle, max_angle, invert: Tham số ánh xạ
        """
        if not (0 <= servo_index < self.num_servos):
            return False
        
        # Tính góc servo mới
        new_angle = self.map_hand_angle_to_servo(
            hand_angle, servo_index, min_angle, max_angle, invert
        )
        
        # Tạo mảng góc mới (copy từ current)
        new_angles = list(self.current_angles)
        new_angles[servo_index] = new_angle
        
        return self.send_angles(new_angles)
    
    def move_to_home(self):
        """Di chuyển tất cả servo về vị trí home (90°)"""
        print("Moving to home position...")
        return self.send_angles([90] * self.num_servos, blocking=True)
    
    def close(self):
        """Đóng kết nối Serial"""
        self.running = False
        
        if self.send_thread:
            self.send_thread.join(timeout=1.0)
        
        if self.connected:
            # Move to safe position before closing
            self.move_to_home()
            time.sleep(0.5)
            self.ser.close()
            print("✓ Serial connection closed")


# ============================================
# EXAMPLE INTEGRATION
# ============================================

if __name__ == "__main__":
    # Test basic functionality
    controller = ServoController(port='COM3', baudrate=115200)
    
    if not controller.connected:
        print("Failed to connect. Exiting...")
        exit(1)
    
    try:
        # Test 1: Home position
        controller.move_to_home()
        time.sleep(1)
        
        # Test 2: Control single servo with hand angle simulation
        print("\nTest: Simulating hand angle control on servo 0")
        for hand_angle in range(0, 181, 30):
            print(f"  Hand angle: {hand_angle}° -> Servo 0")
            controller.control_single_servo(
                servo_index=0,
                hand_angle=hand_angle,
                min_angle=30,   # Khi hand = 0°, servo = 30°
                max_angle=150,  # Khi hand = 180°, servo = 150°
                invert=False
            )
            time.sleep(0.5)
        
        # Test 3: Return home
        print("\nReturning to home...")
        controller.move_to_home()
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        controller.close()
