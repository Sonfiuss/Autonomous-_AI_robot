import cv2
import mediapipe as mp
import math
from ServoController import ServoController

class HandDistanceDetector:
    def __init__(self, enable_servo=False, servo_port='COM3', servo_index=0):
        # Khởi tạo MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # Servo control (optional)
        self.enable_servo = enable_servo
        self.servo_controller = None
        self.servo_index = servo_index
        
        if enable_servo:
            print("\n🔌 Initializing Servo Controller...")
            self.servo_controller = ServoController(port=servo_port, baudrate=115200)
            if self.servo_controller.connected:
                print("✓ Servo control enabled")
                self.servo_controller.move_to_home()
            else:
                print("✗ Servo control disabled (connection failed)")
                self.enable_servo = False
        
    def calculate_angle(self, wrist_pos, thumb_pos, index_pos):
        """Tính góc giữa ngón cái và ngón trỏ (sử dụng cổ tay làm đỉnh)"""
        # Vector từ cổ tay đến ngón cái
        vec1_x = thumb_pos[0] - wrist_pos[0]
        vec1_y = thumb_pos[1] - wrist_pos[1]
        
        # Vector từ cổ tay đến ngón trỏ
        vec2_x = index_pos[0] - wrist_pos[0]
        vec2_y = index_pos[1] - wrist_pos[1]
        
        # Tính góc bằng công thức dot product
        dot_product = vec1_x * vec2_x + vec1_y * vec2_y
        magnitude1 = math.sqrt(vec1_x**2 + vec1_y**2)
        magnitude2 = math.sqrt(vec2_x**2 + vec2_y**2)
        
        # Tránh chia cho 0
        if magnitude1 == 0 or magnitude2 == 0:
            return 0
        
        # Tính cos của góc
        cos_angle = dot_product / (magnitude1 * magnitude2)
        
        # Đảm bảo giá trị nằm trong [-1, 1] để tránh lỗi math domain
        cos_angle = max(-1, min(1, cos_angle))
        
        # Chuyển từ radian sang độ
        angle = math.degrees(math.acos(cos_angle))
        
        return angle
    
    def get_finger_positions(self, hand_landmarks, img_width, img_height):
        """Lấy vị trí pixel của cổ tay, ngón cái và ngón trỏ"""
        # Landmark 0: Cổ tay (Wrist)
        # Landmark 4: Đầu ngón cái (Thumb tip)
        # Landmark 8: Đầu ngón trỏ (Index finger tip)
        
        wrist_lm = hand_landmarks.landmark[0]
        thumb_lm = hand_landmarks.landmark[4]
        index_lm = hand_landmarks.landmark[8]
        
        wrist_pos = (int(wrist_lm.x * img_width), int(wrist_lm.y * img_height))
        thumb_pos = (int(thumb_lm.x * img_width), int(thumb_lm.y * img_height))
        index_pos = (int(index_lm.x * img_width), int(index_lm.y * img_height))
        
        return wrist_pos, thumb_pos, index_pos
    
    def draw_info(self, img, wrist_pos, thumb_pos, index_pos, angle):
        """Vẽ thông tin lên hình ảnh"""
        # Vẽ điểm tại cổ tay
        cv2.circle(img, wrist_pos, 10, (255, 255, 0), cv2.FILLED)
        cv2.putText(img, "Wrist", (wrist_pos[0] - 30, wrist_pos[1] + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        # Vẽ điểm tại ngón cái
        cv2.circle(img, thumb_pos, 10, (255, 0, 0), cv2.FILLED)
        cv2.putText(img, "Thumb", (thumb_pos[0] - 30, thumb_pos[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Vẽ điểm tại ngón trỏ
        cv2.circle(img, index_pos, 10, (0, 255, 0), cv2.FILLED)
        cv2.putText(img, "Index", (index_pos[0] - 30, index_pos[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Vẽ đường từ cổ tay đến ngón cái
        cv2.line(img, wrist_pos, thumb_pos, (255, 0, 255), 2)
        
        # Vẽ đường từ cổ tay đến ngón trỏ
        cv2.line(img, wrist_pos, index_pos, (255, 0, 255), 2)
        
        # Vẽ cung tròn thể hiện góc (tại vị trí cổ tay)
        cv2.ellipse(img, wrist_pos, (50, 50), 0, 0, int(angle), (0, 255, 255), 2)
        
        # Hiển thị góc
        cv2.putText(img, f"Angle: {angle:.1f} deg", 
                    (wrist_pos[0] - 80, wrist_pos[1] - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        return img
    
    def process_frame(self, img):
        """Xử lý một frame"""
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Vẽ các landmarks của bàn tay
                self.mp_draw.draw_landmarks(
                    img, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Lấy vị trí cổ tay, ngón cái và ngón trỏ
                img_height, img_width, _ = img.shape
                wrist_pos, thumb_pos, index_pos = self.get_finger_positions(hand_landmarks, img_width, img_height)
                
                # Tính góc giữa ngón cái và ngón trỏ
                angle = self.calculate_angle(wrist_pos, thumb_pos, index_pos)
                
                # Gửi lệnh điều khiển servo (nếu được bật)
                if self.enable_servo and self.servo_controller:
                    self.servo_controller.control_single_servo(
                        servo_index=self.servo_index,
                        hand_angle=angle,
                        min_angle=30,   # Góc servo khi hand angle = 0°
                        max_angle=150,  # Góc servo khi hand angle = 180°
                        invert=False    # False: angle lớn -> servo lớn
                    )
                
                # Vẽ thông tin lên ảnh
                img = self.draw_info(img, wrist_pos, thumb_pos, index_pos, angle)
                
                # Hiển thị trạng thái (PINCH nếu < 30 độ)
                threshold = 30  # degrees
                status = "PINCH" if angle < threshold else "OPEN"
                color = (0, 255, 0) if status == "PINCH" else (0, 165, 255)
                cv2.putText(img, f"Status: {status}", (10, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)
        
        return img
    
    def run(self):
        """Chạy chương trình chính"""
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        print("=" * 60)
        print("=== Hand Angle Detector ===")
        print("=" * 60)
        print("\nChức năng:")
        print("  ✓ Đo góc giữa ngón cái và ngón trỏ")
        print("  ✓ Sử dụng cổ tay làm điểm gốc")
        print("  ✓ Đơn vị: Độ (Degrees)")
        print("  ✓ PINCH: < 30 độ")
        if self.enable_servo:
            print(f"  ✓ Servo Control: ENABLED (Index {self.servo_index})")
        else:
            print("  ✗ Servo Control: DISABLED")
        print("\nPhím điều khiển:")
        print("  'q': Thoát chương trình")
        print("  's': Chụp và lưu ảnh")
        if self.enable_servo:
            print("  'h': Move servo to home position (90°)")
        print("=" * 60)
        
        while True:
            success, img = cap.read()
            if not success:
                print("Không thể đọc từ camera")
                break
            
            # Lật ảnh để hiển thị như gương
            img = cv2.flip(img, 1)
            
            # Xử lý frame
            img = self.process_frame(img)
            
            # Hiển thị hướng dẫn
            cv2.putText(img, "Press: 'q'=Quit | 's'=Save", 
                       (10, img.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Hiển thị kết quả
            cv2.imshow("Hand Distance Detector", img)
            
            # Xử lý phím bấm
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                # Lưu ảnh
                filename = f"hand_capture_{cv2.getTickCount()}.jpg"
                cv2.imwrite(filename, img)
                print(f"Đã lưu ảnh: {filename}")
            elif key == ord('h') and self.enable_servo:
                # Move servo to home
                print("Moving servo to home position...")
                if self.servo_controller:
                    self.servo_controller.move_to_home()
        
        cap.release()
        cv2.destroyAllWindows()
        self.hands.close()
        
        # Đóng kết nối servo
        if self.enable_servo and self.servo_controller:
            self.servo_controller.close()

def main():
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Hand Angle Detector with Servo Control')
    parser.add_argument('--servo', action='store_true', 
                       help='Enable servo control')
    parser.add_argument('--port', type=str, default='COM3',
                       help='Serial port for Arduino (default: COM3)')
    parser.add_argument('--servo-index', type=int, default=0,
                       help='Servo index to control (0-5, default: 0)')
    args = parser.parse_args()
    
    detector = HandDistanceDetector(
        enable_servo=args.servo,
        servo_port=args.port,
        servo_index=args.servo_index
    )
    detector.run()

if __name__ == "__main__":
    main()
