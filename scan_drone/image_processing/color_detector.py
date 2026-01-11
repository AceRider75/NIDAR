import cv2
import numpy as np
from typing import List, Tuple
import logging

# Handle imports for both package and direct execution
if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    def get_logger():
        return logging.getLogger(__name__)
else:
    try:
        from ..utils.logger import get_logger
    except ImportError:
        logging.basicConfig(level=logging.DEBUG)
        def get_logger():
            return logging.getLogger(__name__)

class ColorDetector:
    """Detects yellow spots in images using HSV color space."""
    
    def __init__(self, lower_yellow: Tuple[int, int, int] = (20, 100, 100),
                 upper_yellow: Tuple[int, int, int] = (30, 255, 255),
                 min_area: int = 500,
                 max_area: int = 50000):
        """
        Initialize color detector.
        
        Args:
            lower_yellow: Lower HSV threshold for yellow detection
            upper_yellow: Upper HSV threshold for yellow detection
            min_area: Minimum contour area to consider as a spot
            max_area: Maximum contour area to consider as a spot
        """
        self.lower_yellow = np.array(lower_yellow)
        self.upper_yellow = np.array(upper_yellow)
        self.min_area = min_area
        self.max_area = max_area
        self.logger = get_logger()
    
    def detect_yellow_spots(self, image: np.ndarray) -> Tuple[List[Tuple[int, int]], List[np.ndarray], np.ndarray]:
        """
        Detect yellow spots in the image.
        
        Args:
            image: Input BGR image
            
        Returns:
            Tuple of (centers, contours, mask)
        """
        try:
            if image is None or image.size == 0:
                self.logger.warning("Empty image received for detection")
                return [], [], np.zeros((100, 100), dtype=np.uint8)
            
            # Convert to HSV color space
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # Create mask for yellow color
            mask = cv2.inRange(hsv_image, self.lower_yellow, self.upper_yellow)
            
            # Apply morphological operations to clean up mask
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            valid_centers = []
            valid_contours = []
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filter by area
                if self.min_area <= area <= self.max_area:
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        valid_centers.append((cX, cY))
                        valid_contours.append(contour)
            
            self.logger.debug(f"Detected {len(valid_centers)} yellow spots")
            return valid_centers, valid_contours, mask
            
        except Exception as e:
            self.logger.error(f"Error in yellow spot detection: {e}")
            return [], [], np.zeros_like(image[:, :, 0]) if image is not None else np.zeros((100, 100), dtype=np.uint8)
    
    def get_spot_info(self, contour: np.ndarray) -> dict:
        """Extract detailed information about a spot."""
        try:
            M = cv2.moments(contour)
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])) if M["m00"] != 0 else (0, 0)
            
            return {
                'center': center,
                'area': area,
                'bounding_box': (x, y, w, h),
                'perimeter': cv2.arcLength(contour, True)
            }
        except Exception as e:
            self.logger.error(f"Error extracting spot info: {e}")
            return {'center': (0, 0), 'area': 0, 'bounding_box': (0, 0, 0, 0), 'perimeter': 0}

    def visualize_detections(self, image: np.ndarray, centers: List[Tuple[int, int]], 
                            contours: List[np.ndarray], mask: np.ndarray) -> np.ndarray:
        """Create an overlay visualization of detected yellow spots."""
        overlay = image.copy()
        
        # Draw contours in green
        cv2.drawContours(overlay, contours, -1, (0, 255, 0), 2)
        
        # Draw centers and labels
        for i, (center, contour) in enumerate(zip(centers, contours)):
            # Get spot info
            info = self.get_spot_info(contour)
            
            # Draw bounding box in blue
            x, y, w, h = info['bounding_box']
            cv2.rectangle(overlay, (x, y), (x + w, y + h), (255, 0, 0), 2)
            
            # Draw centroid with crosshair
            cv2.circle(overlay, center, 8, (0, 0, 255), -1)  # Red filled circle
            cv2.circle(overlay, center, 12, (255, 255, 255), 2)  # White outline
            # Draw crosshair
            cv2.line(overlay, (center[0] - 15, center[1]), (center[0] + 15, center[1]), (255, 255, 255), 2)
            cv2.line(overlay, (center[0], center[1] - 15), (center[0], center[1] + 15), (255, 255, 255), 2)
            
            # Add text labels with outline for visibility
            label = f"Spot {i+1}"
            centroid_label = f"({center[0]}, {center[1]})"
            area_label = f"Area: {info['area']:.0f}px"
            
            # Position labels near the spot
            text_x = center[0] + 20
            text_y = center[1] - 20
            
            # Draw label
            cv2.putText(overlay, label, (text_x, text_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
            cv2.putText(overlay, label, (text_x, text_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Draw centroid coordinates
            cv2.putText(overlay, centroid_label, (text_x, text_y + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
            cv2.putText(overlay, centroid_label, (text_x, text_y + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Draw area
            cv2.putText(overlay, area_label, (text_x, text_y + 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
            cv2.putText(overlay, area_label, (text_x, text_y + 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Print to console
            print(f"Spot {i+1}: Centroid=({center[0]}, {center[1]}), Area={info['area']:.2f}px²")
        
        # Add summary text
        summary = f"Yellow Spots Detected: {len(centers)}"
        cv2.putText(overlay, summary, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 3)
        cv2.putText(overlay, summary, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        return overlay


def main():
    """Test the ColorDetector with camera or image file."""
    detector = ColorDetector()
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Camera not available, using test image...")
        # Create test image with yellow spots
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(image, (200, 200), 40, (0, 255, 255), -1)
        cv2.circle(image, (400, 300), 30, (0, 255, 255), -1)
        cv2.rectangle(image, (100, 350), (150, 400), (0, 255, 255), -1)
        
        centers, contours, mask = detector.detect_yellow_spots(image)
        overlay = detector.visualize_detections(image, centers, contours, mask)
        
        print(f"\nDetected {len(centers)} yellow spot(s):")
        for i, center in enumerate(centers):
            info = detector.get_spot_info(contours[i])
            print(f"  Spot {i+1}: Centroid={center}, Area={info['area']:.2f}px²")
        
        # Create color mask overlay
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_colored[:, :, 1] = mask  # Yellow channel
        mask_colored[:, :, 2] = mask  # Yellow channel
        
        cv2.imshow("Color Feed with Yellow Overlay", overlay)
        cv2.imshow("B&W Mask", mask)
        print("\nPress any key to exit...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("Using camera. Press 'q' to quit...")
        print("Real-time detection active...\n")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            centers, contours, mask = detector.detect_yellow_spots(frame)
            overlay = detector.visualize_detections(frame, centers, contours, mask)
            
            cv2.imshow("Color Feed with Yellow Overlay", overlay)
            cv2.imshow("B&W Mask", mask)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()