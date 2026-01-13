import argparse
import sys
import os
import cv2
import numpy as np
from spot_tracker import SpotTracker
from color_detector import ColorDetector

"""
Test script for SpotTracker - accepts an image and evaluates spot detection performance.

Usage:
    python test_spot_tracker.py <image_path> [--save] [--no-display]
    
Example:
    python test_spot_tracker.py sample_image.jpg --save
"""





def test_spot_tracker_on_image(image_path: str, save_output: bool = False, display: bool = True) -> dict:
    """
    Test the SpotTracker on a single image.
    
    Args:
        image_path: Path to the input image
        save_output: Whether to save the annotated output
        display: Whether to display the result window
        
    Returns:
        Dictionary with detection results
    """
    # Validate image path
    # take relative path and not absolute path
    image_path = os.path.relpath(image_path)

    if not os.path.exists(image_path):
        print(f"[ERROR] Image not found: {image_path}")
        sys.exit(1)
    
    # Load image
    image = cv2.imread(image_path)
    if image is None:
        print(f"[ERROR] Failed to load image: {image_path}")
        sys.exit(1)
    
    height, width = image.shape[:2]
    print(f"\n{'='*60}")
    print(f"SPOT TRACKER TEST")
    print(f"{'='*60}")
    print(f"Image: {image_path}")
    print(f"Size: {width}x{height}")
    print(f"{'='*60}\n")
    
    # Initialize detector and tracker
    detector = ColorDetector()
    tracker = SpotTracker(
        max_distance=80,
        max_frames_missing=5,
        dilation_kernel_size=15,
        clustering_distance=60.0,
        min_cluster_samples=1
    )
    
    # Detect yellow spots
    centers, contours, mask = detector.detect_yellow_spots(image)
    
    print(f"[Detection] Raw detections: {len(centers)} spot(s) found")
    
    # Update tracker
    tracked_spots = tracker.update(centers, contours, mask=mask)
    
    print(f"[Tracking] Tracked spots: {len(tracked_spots)}")
    
    # Print details for each spot
    if tracked_spots:
        print(f"\n{'='*40}")
        print("DETECTED SPOTS:")
        print(f"{'='*40}")
        for spot_id, spot in tracked_spots.items():
            print(f"  Spot ID: {spot.id}")
            print(f"    Center: ({spot.center[0]}, {spot.center[1]})")
            print(f"    Area: {spot.area:.1f} px²")
            print(f"    Rank: #{spot.area_rank}")
            print()
    else:
        print("\n[RESULT] No yellow spots detected in the image.")
    
    # Visualize results
    annotated = tracker.visualize_tracks(image.copy(), show_history=True, show_rank=True)
    
    # Add detection status text
    status_text = f"Detected: {len(tracked_spots)} spot(s)"
    cv2.putText(annotated, status_text, (10, height - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if tracked_spots else (0, 0, 255), 2)
    
    # Also show the mask
    mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    # Create side-by-side comparison
    combined = np.hstack([annotated, mask_colored])
    
    # Save output if requested
    if save_output:
        base_name = os.path.splitext(os.path.basename(image_path))[0]
        output_dir = os.path.dirname(image_path) or "."
        
        annotated_path = os.path.join(output_dir, f"{base_name}_annotated.jpg")
        mask_path = os.path.join(output_dir, f"{base_name}_mask.jpg")
        combined_path = os.path.join(output_dir, f"{base_name}_combined.jpg")
        
        cv2.imwrite(annotated_path, annotated)
        cv2.imwrite(mask_path, mask)
        cv2.imwrite(combined_path, combined)
        
        print(f"\n[Saved] Annotated: {annotated_path}")
        print(f"[Saved] Mask: {mask_path}")
        print(f"[Saved] Combined: {combined_path}")
    
    # Display if requested
    if display:
        cv2.imshow("SpotTracker Test - Annotated | Mask", combined)
        print("\n[INFO] Press any key to close the window...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    # Return results
    results = {
        "image_path": image_path,
        "image_size": (width, height),
        "raw_detections": len(centers),
        "tracked_spots": len(tracked_spots),
        "spots": [
            {
                "id": spot.id,
                "center": spot.center,
                "area": spot.area,
                "rank": spot.area_rank
            }
            for spot in tracked_spots.values()
        ]
    }
    
    print(f"\n{'='*60}")
    print(f"TEST COMPLETE - {'SPOTS DETECTED' if tracked_spots else 'NO SPOTS FOUND'}")
    print(f"{'='*60}\n")
    
    return results


def main():
    parser = argparse.ArgumentParser(
        description="Test SpotTracker on a single image"
    )
    parser.add_argument(
        "image_path",
        help="Path to the input image"
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Save annotated output images"
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Do not display the result window"
    )
    
    args = parser.parse_args()
    
    results = test_spot_tracker_on_image(
        image_path=args.image_path,
        save_output=args.save,
        display=not args.no_display
    )
    
    # Exit with appropriate code
    sys.exit(0 if results["tracked_spots"] > 0 else 1)


if __name__ == "__main__":
    main()