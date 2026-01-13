"""
Main Application
Hand Angle Detection with Real-time Servo Control
"""

import cv2
import argparse
import logging
import time
from typing import Optional

import config
from controllers import HandAngleDetector, ServoController

# Setup logging
logging.basicConfig(
    level=getattr(logging, config.LOG_LEVEL),
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class HandServoApp:
    """Main application class"""
    
    def __init__(self, 
                 enable_servo: bool = False,
                 servo_port: str = None,
                 servo_index: int = 0):
        """
        Initialize application
        
        Args:
            enable_servo: Enable servo control
            servo_port: Serial port for Arduino
            servo_index: Servo index to control
        """
        self.enable_servo = enable_servo
        self.servo_index = servo_index
        
        # Initialize hand detector
        logger.info("Initializing Hand Angle Detector...")
        self.detector = HandAngleDetector()
        
        # Initialize servo controller (optional)
        self.servo = None
        if enable_servo:
            logger.info("Initializing Servo Controller...")
            servo_port = servo_port or config.SERVO_DEFAULT_PORT
            self.servo = ServoController(servo_port)
            
            if not self.servo.connected:
                logger.warning("Servo control disabled (connection failed)")
                self.enable_servo = False
        
        # Initialize camera
        logger.info(f"Initializing camera {config.CAMERA_INDEX}...")
        self.cap = cv2.VideoCapture(config.CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
        
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open camera")
        
        # FPS calculation
        self.fps = 0
        self.frame_count = 0
        self.fps_start_time = time.time()
        
        logger.info("Application initialized successfully")
        self._print_info()
    
    def _print_info(self):
        """Print application information"""
        print("\n" + "=" * 60)
        print("=== Hand Angle Detector with Servo Control ===")
        print("=" * 60)
        print("\n📊 Configuration:")
        print(f"  • Camera: {config.CAMERA_WIDTH}x{config.CAMERA_HEIGHT} @ {config.CAMERA_FPS}fps")
        print(f"  • Angle threshold (PINCH): < {config.ANGLE_PINCH_THRESHOLD}°")
        
        if self.enable_servo:
            print(f"\n🔌 Servo Control: ENABLED")
            print(f"  • Port: {self.servo.port}")
            print(f"  • Controlled servo: #{self.servo_index}")
            print(f"  • Angle range: {config.SERVO_MIN_ANGLE}° - {config.SERVO_MAX_ANGLE}°")
        else:
            print(f"\n✗ Servo Control: DISABLED")
        
        print("\n⌨️  Keyboard Controls:")
        print("  • 'q' - Quit application")
        print("  • 's' - Save screenshot")
        if self.enable_servo:
            print("  • 'h' - Move servo to home position")
            print("  • 'c' - Show servo statistics")
        
        print("=" * 60 + "\n")
    
    def _calculate_fps(self):
        """Calculate and update FPS"""
        self.frame_count += 1
        elapsed = time.time() - self.fps_start_time
        
        if elapsed > 1.0:  # Update every second
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.fps_start_time = time.time()
    
    def _draw_ui(self, img):
        """Draw UI elements on image"""
        # FPS display
        if config.ENABLE_FPS_DISPLAY:
            cv2.putText(img, f"FPS: {self.fps:.1f}", 
                       (img.shape[1] - 150, 30),
                       config.FONT, 0.6, (255, 255, 255), 2)
        
        # Servo status
        if self.enable_servo:
            status_text = f"Servo #{self.servo_index}: ACTIVE"
            cv2.putText(img, status_text, (10, img.shape[0] - 50),
                       config.FONT, 0.6, (0, 255, 0), 2)
        
        # Help text
        help_text = "Press 'q' to quit | 's' to save"
        if self.enable_servo:
            help_text += " | 'h' for home"
        cv2.putText(img, help_text, (10, img.shape[0] - 20),
                   config.FONT, 0.5, (255, 255, 255), 1)
    
    def _handle_keyboard(self, key: int) -> bool:
        """
        Handle keyboard input
        
        Args:
            key: Key code
            
        Returns:
            True to continue, False to quit
        """
        if key == config.KEY_QUIT:
            logger.info("Quit requested by user")
            return False
        
        elif key == config.KEY_SAVE:
            filename = f"hand_capture_{int(time.time())}.jpg"
            cv2.imwrite(filename, self.current_frame)
            logger.info(f"Screenshot saved: {filename}")
            print(f"✓ Saved: {filename}")
        
        elif key == config.KEY_HOME and self.enable_servo:
            logger.info("Moving servo to home position...")
            print("Moving servo to home...")
            if self.servo:
                self.servo.move_to_home()
        
        elif key == config.KEY_CALIBRATE and self.enable_servo:
            if self.servo:
                stats = self.servo.get_statistics()
                print("\n📊 Servo Statistics:")
                for k, v in stats.items():
                    print(f"  • {k}: {v}")
                print()
        
        return True
    
    def run(self):
        """Main application loop"""
        logger.info("Starting main loop...")
        
        try:
            while True:
                # Read frame
                success, img = self.cap.read()
                if not success:
                    logger.error("Failed to read from camera")
                    break
                
                # Flip for mirror effect
                img = cv2.flip(img, 1)
                self.current_frame = img.copy()
                
                # Process frame and detect angle
                angle = self.detector.process_frame(img)
                
                # Control servo if enabled and angle detected
                if self.enable_servo and angle is not None and self.servo:
                    self.servo.control_single_servo(
                        servo_index=self.servo_index,
                        hand_angle=angle
                    )
                
                # Calculate FPS
                self._calculate_fps()
                
                # Draw UI
                self._draw_ui(img)
                
                # Display
                cv2.imshow(config.WINDOW_NAME, img)
                
                # Handle keyboard
                key = cv2.waitKey(1) & 0xFF
                if key != 255:  # Key pressed
                    if not self._handle_keyboard(key):
                        break
        
        except KeyboardInterrupt:
            logger.info("Interrupted by user (Ctrl+C)")
        
        except Exception as e:
            logger.error(f"Error in main loop: {e}", exc_info=True)
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        logger.info("Cleaning up...")
        
        if self.cap:
            self.cap.release()
        
        cv2.destroyAllWindows()
        
        if self.detector:
            self.detector.close()
        
        if self.servo:
            self.servo.close()
        
        logger.info("Cleanup complete")


def main():
    """Entry point"""
    parser = argparse.ArgumentParser(
        description='Hand Angle Detection with Real-time Servo Control',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic detection (no servo)
  python main.py
  
  # With servo control
  python main.py --servo --port COM3 --servo-index 0
  
  # Linux/Mac
  python main.py --servo --port /dev/ttyUSB0 --servo-index 0
        """
    )
    
    parser.add_argument('--servo', action='store_true',
                       help='Enable servo control')
    parser.add_argument('--port', type=str, default=None,
                       help=f'Serial port (default: {config.SERVO_DEFAULT_PORT})')
    parser.add_argument('--servo-index', type=int, default=0,
                       help='Servo index to control (0-5, default: 0)')
    parser.add_argument('--debug', action='store_true',
                       help='Enable debug logging')
    
    args = parser.parse_args()
    
    # Set debug level if requested
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Create and run application
    app = HandServoApp(
        enable_servo=args.servo,
        servo_port=args.port,
        servo_index=args.servo_index
    )
    
    app.run()


if __name__ == "__main__":
    main()
