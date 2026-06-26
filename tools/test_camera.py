#!/usr/bin/env python3
"""
tools/test_camera.py
====================
Quick standalone test: opens the Pi Camera via V4L2, runs YOLO on CPU,
prints detections and optionally shows a preview window.

No MAVLink required.  Use this to verify your model and camera work
before mounting on the drone.

Usage (from project root):
    python tools/test_camera.py --model models/drone_yolo.pt --display
"""

import sys
import time
import argparse

# Allow running from the project root without installing as a package
sys.path.insert(0, ".")

import cv2
from ultralytics import YOLO


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Camera + YOLO smoke test (hardware mode only)"
    )
    parser.add_argument("--model",   default="models/drone_yolo.pt",
                        help="Path to YOLO .pt model weights")
    parser.add_argument("--conf",    type=float, default=0.5,
                        help="Detection confidence threshold")
    parser.add_argument("--width",   type=int, default=640)
    parser.add_argument("--height",  type=int, default=480)
    parser.add_argument("--display", action="store_true", default=True,
                        help="Show OpenCV preview window (default: True)")
    args = parser.parse_args()

    print(f"Loading model: {args.model}")
    model = YOLO(args.model)

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("ERROR: Cannot open camera at /dev/video0")
        sys.exit(1)

    print("Camera open — press Q to quit")
    t0     = time.monotonic()
    frames = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame read failed")
            break

        results = model.predict(
            frame, imgsz=320, conf=args.conf, verbose=False, device="cpu"
        )

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls  = int(box.cls[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                print(f"  cls={cls} conf={conf:.2f} center=({cx},{cy})")
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 255), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 200, 255), -1)

        frames  += 1
        elapsed  = time.monotonic() - t0
        fps      = frames / elapsed if elapsed > 0 else 0
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

        if args.display:
            cv2.imshow("Camera Test", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()
    print(f"Final average FPS: {frames / (time.monotonic() - t0):.1f}")


if __name__ == "__main__":
    main()
