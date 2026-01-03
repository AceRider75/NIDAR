import cv2
import numpy as np
import os, sys

# --- Path Setup ---
BASE_DIR = os.path.dirname(os.path.dirname(__file__)) if '__file__' in locals() else os.getcwd()
UTILS_DIR = os.path.join(BASE_DIR, 'src/utils')
if UTILS_DIR not in sys.path:
    sys.path.insert(0, UTILS_DIR)

# Attempt to import your custom curve utils
try:
    import utils.curve_fit as curve_utils
except ImportError:
    curve_utils = None
    print("Warning: utils.curve_fit not found. Falling back to standard contours.")

class LargestSpotTracker:
    def __init__(self, hsv_lower=(20, 100, 100), hsv_upper=(35, 255, 255), min_area=250, smooth_alpha=0.6):
        self.hsv_lower = np.array(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper, dtype=np.uint8)
        self.min_area = min_area
        self.smooth_alpha = smooth_alpha
        self.prev_center = None

    def _preprocess(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask

    def _largest_contour(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_area:
            return None
        return largest

    def process(self, frame):
        mask = self._preprocess(frame)
        cnt = self._largest_contour(mask)
        info = {"found": False, "center": None, "bbox": None, "area": 0}

        if cnt is not None:
            area = cv2.contourArea(cnt)
            x, y, w, h = cv2.boundingRect(cnt)
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = x + w // 2, y + h // 2

            if self.prev_center is None:
                center = (cx, cy)
            else:
                px, py = self.prev_center
                center = (int(self.smooth_alpha * px + (1 - self.smooth_alpha) * cx),
                          int(self.smooth_alpha * py + (1 - self.smooth_alpha) * cy))
            self.prev_center = center
            info.update({"found": True, "center": center, "bbox": (x, y, w, h), "area": int(area)})

            # Draw overlays
            if curve_utils:
                curve_pts = curve_utils.fit_smooth_closed_curve(cnt, num_points=150, simplify_epsilon=2.0, chaikin_iters=2)
                curve_utils.draw_curve(frame, curve_pts, color=(0, 255, 0), thickness=2)
            else:
                cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)

            cv2.circle(frame, center, 6, (0, 0, 255), -1)
            cv2.putText(frame, f"Area:{int(area)}", (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        else:
            if self.prev_center is not None:
                cx, cy = self.prev_center
                cv2.circle(frame, (cx, cy), 6, (0, 165, 255), 2)
                cv2.putText(frame, "Lost (holding last)", (cx + 8, cy - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)

        return frame, mask, info

def main():
    cap = cv2.VideoCapture(0)
    tracker = LargestSpotTracker()

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # --- Video Writer Setup ---
    # Define codec and get frame properties
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 0: fps = 20.0  # Default if webcam doesn't report FPS
    
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    size = (width, height)

    # Initialize two writers
    out_orig = cv2.VideoWriter('original_output.avi', fourcc, fps, size)
    out_annot = cv2.VideoWriter('annotated_output.avi', fourcc, fps, size)

    print(f"Recording at {width}x{height} @ {fps} FPS. Press 'q' to stop.")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        # 1. Capture the raw frame BEFORE the tracker modifies it
        original_frame = frame.copy()
        
        # 2. Process the frame (this modifies 'frame' in-place)
        frame_annotated, mask, info = tracker.process(frame)

        # 3. Save both versions
        out_orig.write(original_frame)
        out_annot.write(frame_annotated)

        # Display results
        cv2.imshow("Annotated (Saving...)", frame_annotated)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    cap.release()
    out_orig.release()
    out_annot.release()
    cv2.destroyAllWindows()
    print("Videos saved as 'original_output.avi' and 'annotated_output.avi'")

if __name__ == "__main__":
    main()