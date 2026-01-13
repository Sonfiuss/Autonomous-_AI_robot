"""
Hand Angle Detector
Detects hand landmarks and calculates angle between thumb and index finger
"""

import cv2
import mediapipe as mp
import math
import logging
from typing import Optional, Tuple

import config

logger = logging.getLogger(__name__)


class HandAngleDetector:
    """
    Hand angle detection using MediaPipe
    Calculates angle between thumb tip, wrist, and index finger tip
    """
    
    def __init__(self):
        """Initialize MediaPipe Hands"""
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=config.MEDIAPIPE_STATIC_IMAGE_MODE,
            max_num_hands=config.MEDIAPIPE_MAX_HANDS,
            min_detection_confidence=config.MEDIAPIPE_MIN_DETECTION_CONFIDENCE,
            min_tracking_confidence=config.MEDIAPIPE_MIN_TRACKING_CONFIDENCE
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        logger.info("Hand angle detector initialized")
    
    def calculate_angle(self, 
                       wrist_pos: Tuple[int, int],
                       thumb_pos: Tuple[int, int],
                       index_pos: Tuple[int, int]) -> float:
        """
        Calculate angle between thumb and index finger using wrist as vertex
        
        Args:
            wrist_pos: (x, y) position of wrist
            thumb_pos: (x, y) position of thumb tip
            index_pos: (x, y) position of index finger tip
            
        Returns:
            Angle in degrees (0-180)
        """
        # Vector from wrist to thumb
        vec1_x = thumb_pos[0] - wrist_pos[0]
        vec1_y = thumb_pos[1] - wrist_pos[1]
        
        # Vector from wrist to index
        vec2_x = index_pos[0] - wrist_pos[0]
        vec2_y = index_pos[1] - wrist_pos[1]
        
        # Calculate angle using dot product
        dot_product = vec1_x * vec2_x + vec1_y * vec2_y
        magnitude1 = math.sqrt(vec1_x**2 + vec1_y**2)
        magnitude2 = math.sqrt(vec2_x**2 + vec2_y**2)
        
        # Avoid division by zero
        if magnitude1 == 0 or magnitude2 == 0:
            return 0
        
        # Calculate cosine of angle
        cos_angle = dot_product / (magnitude1 * magnitude2)
        
        # Clamp to valid range to avoid math domain error
        cos_angle = max(-1, min(1, cos_angle))
        
        # Convert from radians to degrees
        angle = math.degrees(math.acos(cos_angle))
        
        return angle
    
    def get_finger_positions(self, 
                            hand_landmarks,
                            img_width: int,
                            img_height: int) -> Tuple[Tuple[int, int], ...]:
        """
        Extract pixel positions of wrist, thumb tip, and index finger tip
        
        Args:
            hand_landmarks: MediaPipe hand landmarks
            img_width: Image width
            img_height: Image height
            
        Returns:
            Tuple of (wrist_pos, thumb_pos, index_pos)
        """
        # Landmark indices:
        # 0: Wrist
        # 4: Thumb tip
        # 8: Index finger tip
        
        wrist_lm = hand_landmarks.landmark[0]
        thumb_lm = hand_landmarks.landmark[4]
        index_lm = hand_landmarks.landmark[8]
        
        wrist_pos = (int(wrist_lm.x * img_width), int(wrist_lm.y * img_height))
        thumb_pos = (int(thumb_lm.x * img_width), int(thumb_lm.y * img_height))
        index_pos = (int(index_lm.x * img_width), int(index_lm.y * img_height))
        
        return wrist_pos, thumb_pos, index_pos
    
    def draw_annotations(self,
                        img,
                        wrist_pos: Tuple[int, int],
                        thumb_pos: Tuple[int, int],
                        index_pos: Tuple[int, int],
                        angle: float):
        """
        Draw visual annotations on image
        
        Args:
            img: Input image
            wrist_pos: Wrist position
            thumb_pos: Thumb tip position
            index_pos: Index finger tip position
            angle: Calculated angle
        """
        # Draw wrist
        cv2.circle(img, wrist_pos, 10, config.COLOR_WRIST, cv2.FILLED)
        cv2.putText(img, "Wrist", (wrist_pos[0] - 30, wrist_pos[1] + 30),
                    config.FONT, 0.5, config.COLOR_WRIST, 2)
        
        # Draw thumb
        cv2.circle(img, thumb_pos, 10, config.COLOR_THUMB, cv2.FILLED)
        cv2.putText(img, "Thumb", (thumb_pos[0] - 30, thumb_pos[1] - 20),
                    config.FONT, 0.5, config.COLOR_THUMB, 2)
        
        # Draw index finger
        cv2.circle(img, index_pos, 10, config.COLOR_INDEX, cv2.FILLED)
        cv2.putText(img, "Index", (index_pos[0] - 30, index_pos[1] - 20),
                    config.FONT, 0.5, config.COLOR_INDEX, 2)
        
        # Draw lines
        cv2.line(img, wrist_pos, thumb_pos, config.COLOR_LINE, 2)
        cv2.line(img, wrist_pos, index_pos, config.COLOR_LINE, 2)
        
        # Draw angle arc
        cv2.ellipse(img, wrist_pos, (50, 50), 0, 0, int(angle), config.COLOR_ANGLE, 2)
        
        # Display angle value
        cv2.putText(img, f"Angle: {angle:.1f} deg",
                    (wrist_pos[0] - 80, wrist_pos[1] - 40),
                    config.FONT, 0.8, config.COLOR_TEXT, 2)
    
    def process_frame(self, img) -> Optional[float]:
        """
        Process a single frame and detect hand angle
        
        Args:
            img: Input image (BGR)
            
        Returns:
            Detected angle or None if no hand detected
        """
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)
        
        detected_angle = None
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw hand landmarks
                self.mp_draw.draw_landmarks(
                    img, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Get positions
                img_height, img_width, _ = img.shape
                wrist_pos, thumb_pos, index_pos = self.get_finger_positions(
                    hand_landmarks, img_width, img_height
                )
                
                # Calculate angle
                angle = self.calculate_angle(wrist_pos, thumb_pos, index_pos)
                detected_angle = angle
                
                # Draw annotations
                self.draw_annotations(img, wrist_pos, thumb_pos, index_pos, angle)
                
                # Display status
                threshold = config.ANGLE_PINCH_THRESHOLD
                status = "PINCH" if angle < threshold else "OPEN"
                color = config.COLOR_PINCH if status == "PINCH" else config.COLOR_OPEN
                cv2.putText(img, f"Status: {status}", (10, 50),
                           config.FONT, 1, color, 3)
        
        return detected_angle
    
    def close(self):
        """Release resources"""
        self.hands.close()
        logger.info("Hand detector closed")
