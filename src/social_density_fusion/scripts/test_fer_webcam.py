#!/usr/bin/env python3
"""Test FER trên webcam IdeaPad (YuNet + alignment + group scoring).

Chạy:
  python3 test_fer_webcam.py --show              # preview live
  python3 test_fer_webcam.py --frames 60         # test nhanh
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import cv2

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from emotion_fer_backend import FerPlusBackend, YuNetFaceDetector  # noqa: E402


def format_group_scores(groups: dict) -> str:
    return "H:{:.0f}% A:{:.0f}% N:{:.0f}%".format(
        groups.get("Happy", 0) * 100,
        groups.get("Angry", 0) * 100,
        groups.get("Neutral", 0) * 100,
    )


def main():
    parser = argparse.ArgumentParser(description="Test FER webcam (YuNet + alignment)")
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--save", type=str, default="")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--model", type=str, default="")
    parser.add_argument("--group-margin", type=float, default=0.12)
    args = parser.parse_args()

    backend = FerPlusBackend(args.model or None, group_margin=args.group_margin)
    try:
        detector = YuNetFaceDetector()
        detect_mode = "YuNet"
    except FileNotFoundError:
        from test_fer_webcam import load_face_cascade, detect_faces  # noqa: WPS433

        cascade, _ = load_face_cascade()
        detector = cascade
        detect_mode = "Haar"

    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        print("[FAIL] Không mở webcam device={}".format(args.device))
        return 1

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print("[OK] Webcam {}x{} | detector={} | margin={}".format(w, h, detect_mode, args.group_margin))

    frame_count = detect_count = 0
    emotion_counts = {"Happy": 0, "Angry": 0, "Neutral": 0}
    last_annotated = None
    t0 = time.time()

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame_count += 1
            annotated = frame.copy()

            if detect_mode == "YuNet":
                faces = detector.detect(frame)
                if faces:
                    face_row = faces[0]
                    crop = detector.crop_aligned(frame, face_row)
                    if crop is not None:
                        social, raw, conf, groups = backend.predict_detailed(crop)
                        detect_count += 1
                        emotion_counts[social] = emotion_counts.get(social, 0) + 1
                        x, y, fw, fh = map(int, face_row[:4])
                        cv2.rectangle(annotated, (x, y), (x + fw, y + fh), (0, 255, 0), 2)
                        label = "{} ({}) {:.0f}%".format(social, raw, conf * 100)
                        cv2.putText(annotated, label, (x, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
                        cv2.putText(
                            annotated,
                            format_group_scores(groups),
                            (x, y + fh + 18),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.45,
                            (255, 200, 0),
                            1,
                        )
                        print("[{:3d}] {} | {} | {}".format(frame_count, label, format_group_scores(groups), raw))
            else:
                import numpy as np  # noqa: WPS433

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                boxes = detect_faces(gray, detector)
                if boxes:
                    x, y, fw, fh = boxes[0]
                    crop = frame[y : y + fh, x : x + fw]
                    social, raw, conf, groups = backend.predict_detailed(crop)
                    detect_count += 1
                    emotion_counts[social] = emotion_counts.get(social, 0) + 1
                    cv2.rectangle(annotated, (x, y), (x + fw, y + fh), (0, 255, 0), 2)

            last_annotated = annotated
            if args.show:
                cv2.imshow("FER webcam (YuNet + alignment)", annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            if args.frames > 0 and frame_count >= args.frames:
                break
    finally:
        cap.release()
        if args.show:
            cv2.destroyAllWindows()

    elapsed = time.time() - t0
    print("\n===== Kết quả =====")
    print("Frame: {} | Detect: {} ({:.0f}%) | {:.1f} FPS".format(
        frame_count, detect_count, 100 * detect_count / max(1, frame_count), frame_count / max(0.001, elapsed)
    ))
    print("Phân bố:", emotion_counts)
    if args.save and last_annotated is not None:
        cv2.imwrite(args.save, last_annotated)
        print("[OK] Lưu:", args.save)
    return 0 if detect_count > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
