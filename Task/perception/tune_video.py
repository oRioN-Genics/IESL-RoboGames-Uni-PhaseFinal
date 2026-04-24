import cv2
import numpy as np
import sys
import os

# Try/except block to handle both absolute and relative import paths
try:
    from perception.preprocessor import Preprocessor, PreprocessorConfig
    from perception.apriltag_detector import AprilTagDetector
except ImportError:
    from preprocessor import Preprocessor, PreprocessorConfig
    from apriltag_detector import AprilTagDetector

def draw_tags_on_frame(vis, tags):
    """Helper function to draw bounding boxes and decoded text over AprilTags."""
    for tag in tags:
        corners = tag.corners.astype(int)
        for i in range(4):
            cv2.line(vis, tuple(corners[i]), tuple(
                corners[(i + 1) % 4]), (0, 255, 255), 2)
        cx, cy = int(tag.center[0]), int(tag.center[1])
        cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)
        
        # Display the decoded Country Code and Airport Status
        label = f"ID:{tag.tag_id} C:{tag.country_code} S:{tag.airport_status}"
        cv2.putText(vis, label, (cx - 40, cy - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 2)

def run_video_tuner(video_path: str):
    cap = cv2.VideoCapture(video_path)
    
    if not cap.isOpened():
        print(f"Error: Could not open video {video_path}")
        return

    # Create two windows: one for the Line tuner, one for the Tag detector
    cv2.namedWindow("HSV Tuner", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Full Frame + Tags", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Full Frame + Tags", 640, 480)
    
    # --- RED VALUES ---
    for name, val in [("H low", 0), ("H high", 15),
                      ("S low", 100), ("S high", 255),
                      ("V low", 60), ("V high", 255)]:
        cv2.createTrackbar(name, "HSV Tuner", val, 255, lambda x: None)

    pre = Preprocessor(PreprocessorConfig())
    
    # Initialize the exact AprilTag settings used in flight.py
    tag_detector = AprilTagDetector(
        quad_decimate=2.0,
        quad_sigma=0.2,
        decode_sharpening=0.3,
        nthreads=2,
    )

    print(f"[Tuner] Playing {video_path}. Adjust sliders.")
    print("[Tuner] SPACE to pause/play. Q to quit and print final values.")
    
    paused = False
    raw_frame = None

    while True:
        if not paused:
            ret, new_frame = cap.read()
            
            # Loop the video continuously if it reaches the end
            if not ret:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
                
            raw_frame = new_frame

        if raw_frame is None:
            continue

        # 1. Resize to match physical drone camera (CRITICAL FOR ACCURATE SIMULATION)
        frame = cv2.resize(raw_frame, (640, 480))
        display_frame = frame.copy()

        # 2. AprilTag Detection Pipeline
        gray = cv2.cvtColor(display_frame, cv2.COLOR_BGR2GRAY)
        tags = tag_detector.detect(gray)
        draw_tags_on_frame(display_frame, tags)

        # 3. Line Detection Pipeline
        h_lo = cv2.getTrackbarPos("H low",  "HSV Tuner")
        h_hi = cv2.getTrackbarPos("H high", "HSV Tuner")
        s_lo = cv2.getTrackbarPos("S low",  "HSV Tuner")
        s_hi = cv2.getTrackbarPos("S high", "HSV Tuner")
        v_lo = cv2.getTrackbarPos("V low",  "HSV Tuner")
        v_hi = cv2.getTrackbarPos("V high", "HSV Tuner")

        pre.update_hsv([h_lo, s_lo, v_lo], [h_hi, s_hi, v_hi])
        
        # We pass the display_frame so you can see if the tags interfere with the line crop
        mask, roi = pre.process(display_frame)

        # Display the color crop and the binary mask side-by-side
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined_view = np.hstack([roi, mask_bgr])
        
        # Show both windows
        cv2.imshow("HSV Tuner", combined_view)
        cv2.imshow("Full Frame + Tags", display_frame)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            print(f"\n[Tuner] Final HSV values to put in your PreprocessorConfig:")
            print(f"  hsv_lower: [{h_lo}, {s_lo}, {v_lo}]")
            print(f"  hsv_upper: [{h_hi}, {s_hi}, {v_hi}]")
            break
        elif key == ord(' '):
            paused = not paused

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Change Uni-PhaseFinal back to Uni-Finals
    raw_path = "~/IESL-RoboGames-Uni-Finals/samples/video/line_follow.mp4" 
    absolute_path = os.path.expanduser(raw_path)
    run_video_tuner(absolute_path)