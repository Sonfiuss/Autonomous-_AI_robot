"""
Angle Mapping Utilities
Convert between hand angles and servo angles
"""

from typing import List


class AngleMapper:
    """Maps hand angles to servo angles with configurable ranges"""
    
    @staticmethod
    def map_angle(hand_angle: float,
                  min_angle: float = 0,
                  max_angle: float = 180,
                  invert: bool = False) -> float:
        """
        Map hand angle (0-180°) to servo angle
        
        Args:
            hand_angle: Input angle from hand detection (0-180°)
            min_angle: Servo angle when hand_angle = 0°
            max_angle: Servo angle when hand_angle = 180°
            invert: Reverse the mapping
            
        Returns:
            Mapped servo angle
        """
        # Clamp input
        hand_angle = max(0, min(180, hand_angle))
        
        # Calculate ratio
        if invert:
            ratio = 1.0 - (hand_angle / 180.0)
        else:
            ratio = hand_angle / 180.0
        
        # Map to servo range
        servo_angle = min_angle + ratio * (max_angle - min_angle)
        
        # Clamp output
        return max(0, min(180, servo_angle))
    
    @staticmethod
    def clamp_angles(angles: List[float]) -> List[float]:
        """
        Clamp all angles to valid range [0, 180]
        
        Args:
            angles: List of angles
            
        Returns:
            Clamped angles
        """
        return [max(0, min(180, angle)) for angle in angles]
    
    @staticmethod
    def interpolate(current: float, target: float, alpha: float = 0.2) -> float:
        """
        Smooth interpolation between current and target angle
        
        Args:
            current: Current angle
            target: Target angle
            alpha: Interpolation factor (0-1), higher = faster
            
        Returns:
            Interpolated angle
        """
        return current + alpha * (target - current)
