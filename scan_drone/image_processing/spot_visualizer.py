"""
Interactive visualizer for tuning ColorDetector thresholds and SpotTracker parameters.
Set `IMAGE_PATH` below to the file you want to test.

Usage: edit IMAGE_PATH and run
    python3 spot_visualizer.py

Controls:
 - Trackbars adjust HSV lower/upper and min area
 - Press 's' to save annotated + mask images next to the source
 - Press 'q' or Esc to quit
"""

import os
import cv2
import numpy as np
from color_detector import ColorDetector
from spot_tracker import SpotTracker

# === User-editable path ===
IMAGE_PATH = "WhatsApp Image 2026-01-12 at 17.35.58.jpeg"  # change to your image
# ===========================

WINDOW_NAME = "Spot Visualizer"

# Default values (match ColorDetector defaults)
init_vals = {
    'h_low': 20,
    's_low': 15,
    'v_low': 200,
    'h_high': 36,
    's_high': 240,
    'v_high': 255,
    'min_area': 400,
}

# Helpers for trackbars
def nothing(x):
    pass

def create_trackbar_window():
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.createTrackbar('H Low', WINDOW_NAME, init_vals['h_low'], 179, nothing)
    cv2.createTrackbar('S Low', WINDOW_NAME, init_vals['s_low'], 255, nothing)
    cv2.createTrackbar('V Low', WINDOW_NAME, init_vals['v_low'], 255, nothing)
    cv2.createTrackbar('H High', WINDOW_NAME, init_vals['h_high'], 179, nothing)
    cv2.createTrackbar('S High', WINDOW_NAME, init_vals['s_high'], 255, nothing)
    cv2.createTrackbar('V High', WINDOW_NAME, init_vals['v_high'], 255, nothing)
    cv2.createTrackbar('Min Area', WINDOW_NAME, init_vals['min_area'], 10000, nothing)


def read_trackbar_values():
    hl = cv2.getTrackbarPos('H Low', WINDOW_NAME)
    sl = cv2.getTrackbarPos('S Low', WINDOW_NAME)
    vl = cv2.getTrackbarPos('V Low', WINDOW_NAME)
    hh = cv2.getTrackbarPos('H High', WINDOW_NAME)
    sh = cv2.getTrackbarPos('S High', WINDOW_NAME)
    vh = cv2.getTrackbarPos('V High', WINDOW_NAME)
    ma = cv2.getTrackbarPos('Min Area', WINDOW_NAME)
    return (hl, sl, vl), (hh, sh, vh), max(1, ma)


def main():
    # Resolve image path
    img_path = os.path.relpath(IMAGE_PATH)
    if not os.path.exists(img_path):
        print(f"[ERROR] Image not found: {img_path}")
        return

    image = cv2.imread(img_path)
    if image is None:
        print(f"[ERROR] Failed to load image: {img_path}")
        return

    detector = ColorDetector()
    # Disable clustering/merge for tuning/visualization
    tracker = SpotTracker(max_distance=80, max_frames_missing=5, dilation_kernel_size=0,
                          clustering_distance=60.0, min_cluster_samples=1, use_clustering=False)

    create_trackbar_window()

    while True:
        # Read trackbar values and apply to detector
        lower, upper, min_area = read_trackbar_values()
        detector.lower_yellow = np.array(lower, dtype=np.uint8)
        detector.upper_yellow = np.array(upper, dtype=np.uint8)
        detector.min_area = min_area

        centers, contours, mask = detector.detect_yellow_spots(image)

        # Reset tracker and update once for visualization
        tracker.reset()
        tracker.update(centers, contours, mask=mask)

        annotated = tracker.visualize_tracks(image.copy(), show_history=False, show_rank=True)
        overlay = detector.visualize_detections(image.copy(), centers, contours, mask)

        # Stack annotated (tracker) and overlay (detector) and mask
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        top = np.hstack([annotated, overlay])
        bottom = np.hstack([mask_bgr, np.zeros_like(mask_bgr)])
        # Resize bottom to match top height if necessary
        if bottom.shape[1] != top.shape[1]:
            bottom = cv2.resize(bottom, (top.shape[1], top.shape[0]))

        combined = np.vstack([top, bottom])

        cv2.imshow(WINDOW_NAME, combined)

        key = cv2.waitKey(100) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif key == ord('s'):
            # Save annotated outputs
            base = os.path.splitext(os.path.basename(img_path))[0]
            out_dir = os.path.dirname(img_path) or '.'
            annotated_path = os.path.join(out_dir, f"{base}_annotated_tracker.jpg")
            overlay_path = os.path.join(out_dir, f"{base}_annotated_detector.jpg")
            mask_path = os.path.join(out_dir, f"{base}_mask.jpg")
            cv2.imwrite(annotated_path, annotated)
            cv2.imwrite(overlay_path, overlay)
            cv2.imwrite(mask_path, mask)
            print(f"Saved: {annotated_path}, {overlay_path}, {mask_path}")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
