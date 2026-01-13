"""
Servo Controller
High-level servo control interface with angle mapping
"""

import logging
from typing import List, Optional

from communication import ArduinoSerialComm
from utils import AngleMapper
import config

logger = logging.getLogger(__name__)


class ServoController:
    """
    High-level servo controller
    Manages servo angles and communication with Arduino
    """
    
    def __init__(self, port: str, baudrate: int = None, num_servos: int = None):
        """
        Initialize servo controller
        
        Args:
            port: Serial port
            baudrate: Baud rate (default from config)
            num_servos: Number of servos (default from config)
        """
        self.port = port
        self.baudrate = baudrate or config.SERVO_BAUDRATE
        self.num_servos = num_servos or config.SERVO_NUM_SERVOS
        
        # Initialize serial communication
        self.comm = ArduinoSerialComm(self.port, self.baudrate)
        self.connected = self.comm.connected
        
        # Current servo angles
        self.current_angles = [config.SERVO_HOME_ANGLE] * self.num_servos
        
        # Angle mapper utility
        self.mapper = AngleMapper()
        
        if self.connected:
            logger.info(f"Servo controller initialized with {self.num_servos} servos")
            self.move_to_home()
        else:
            logger.warning("Servo controller initialized but not connected")
    
    def _format_command(self, angles: List[float]) -> str:
        """
        Format angles into Arduino command
        
        Args:
            angles: List of servo angles
            
        Returns:
            Formatted command string
        """
        # Format: <angle0,angle1,...,angleN>\n
        angles_str = ','.join([f"{a:.1f}" for a in angles])
        return f"<{angles_str}>\n"
    
    def send_angles(self, angles: List[float], blocking: bool = False) -> bool:
        """
        Send angle commands to all servos
        
        Args:
            angles: List of angles (must match num_servos)
            blocking: Wait for command to be sent
            
        Returns:
            True if successful
        """
        if not self.connected:
            return False
        
        if len(angles) != self.num_servos:
            logger.error(f"Expected {self.num_servos} angles, got {len(angles)}")
            return False
        
        # Clamp angles to valid range
        clamped = self.mapper.clamp_angles(angles)
        
        # Format and send command
        command = self._format_command(clamped)
        success = self.comm.send_command(command, blocking)
        
        if success:
            self.current_angles = clamped
        
        return success
    
    def control_single_servo(self,
                            servo_index: int,
                            hand_angle: float,
                            min_angle: float = None,
                            max_angle: float = None,
                            invert: bool = None) -> bool:
        """
        Control a single servo based on hand angle
        Other servos maintain their current positions
        
        Args:
            servo_index: Index of servo to control (0 to num_servos-1)
            hand_angle: Detected hand angle (0-180°)
            min_angle: Minimum servo angle (default from config)
            max_angle: Maximum servo angle (default from config)
            invert: Invert mapping (default from config)
            
        Returns:
            True if successful
        """
        if not (0 <= servo_index < self.num_servos):
            logger.error(f"Invalid servo index: {servo_index}")
            return False
        
        # Use config defaults if not specified
        min_angle = min_angle if min_angle is not None else config.SERVO_MIN_ANGLE
        max_angle = max_angle if max_angle is not None else config.SERVO_MAX_ANGLE
        invert = invert if invert is not None else config.SERVO_INVERT
        
        # Map hand angle to servo angle
        servo_angle = self.mapper.map_angle(hand_angle, min_angle, max_angle, invert)
        
        # Create new angle array (copy current angles)
        new_angles = list(self.current_angles)
        new_angles[servo_index] = servo_angle
        
        return self.send_angles(new_angles)
    
    def move_to_home(self) -> bool:
        """
        Move all servos to home position
        
        Returns:
            True if successful
        """
        logger.info("Moving servos to home position...")
        home_angles = [config.SERVO_HOME_ANGLE] * self.num_servos
        return self.send_angles(home_angles, blocking=True)
    
    def get_statistics(self) -> dict:
        """Get communication statistics"""
        stats = self.comm.get_statistics()
        stats['current_angles'] = self.current_angles
        return stats
    
    def close(self):
        """Close servo controller and move to safe position"""
        if self.connected:
            logger.info("Closing servo controller...")
            self.move_to_home()
            self.comm.close()
