#!/usr/bin/env python3
"""Backend FER+ ONNX qua OpenCV DNN (CPU).

Cải tiến độ chính xác:
- Căn mặt theo 2 mắt (face alignment) trước khi infer
- CLAHE cân bằng sáng
- Gom xác suất theo nhóm Happy/Angry/Neutral (thay vì argmax 1 class)
- Ngưỡng margin: chỉ đổi emotion khi nhóm thắng rõ ràng
"""

from __future__ import annotations

import math
import os
from typing import Dict, Optional, Sequence, Tuple

import cv2
import numpy as np
import rospkg

FERPLUS_LABELS = (
    "neutral",
    "happiness",
    "surprise",
    "sadness",
    "anger",
    "disgust",
    "fear",
    "contempt",
)

FERPLUS_TO_SOCIAL = {
    "neutral": "Neutral",
    "happiness": "Happy",
    "surprise": "Happy",
    "sadness": "Neutral",
    "anger": "Angry",
    "disgust": "Angry",
    "fear": "Angry",
    "contempt": "Angry",
}

SOCIAL_GROUPS = {
    "Happy": ("happiness", "surprise"),
    "Angry": ("anger", "disgust", "fear", "contempt"),
    "Neutral": ("neutral", "sadness"),
}

# Trọng số class giận — anger/contempt quan trọng nhất cho Track D
ANGRY_CLASS_WEIGHTS = {
    "anger": 1.6,
    "disgust": 1.3,
    "contempt": 1.4,
    "fear": 0.9,
}

BODY34_HEAD = 26
BODY34_NOSE = 27
BODY34_LEFT_EYE = 28
BODY34_RIGHT_EYE = 30

BODY18_NOSE = 0
BODY18_LEFT_EYE = 15
BODY18_RIGHT_EYE = 14


def default_model_path() -> str:
    try:
        pkg = rospkg.RosPack().get_path("fer_emotion")
    except rospkg.ResourceNotFound:
        pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(pkg, "models", "emotion-ferplus-8.onnx")


def default_yunet_path() -> str:
    try:
        pkg = rospkg.RosPack().get_path("fer_emotion")
    except rospkg.ResourceNotFound:
        pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(pkg, "models", "face_detection_yunet_2023mar.onnx")


def softmax(logits: np.ndarray) -> np.ndarray:
    x = logits.astype(np.float64)
    x = x - np.max(x)
    e = np.exp(x)
    return e / np.sum(e)


def preprocess_face_gray(face_bgr: np.ndarray, use_clahe: bool = True) -> np.ndarray:
    gray = cv2.cvtColor(face_bgr, cv2.COLOR_BGR2GRAY)
    if use_clahe:
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
        gray = clahe.apply(gray)
    return gray


def align_face_by_eyes(
    face_bgr: np.ndarray,
    left_eye: Tuple[float, float],
    right_eye: Tuple[float, float],
    output_size: int = 128,
) -> np.ndarray:
    """Xoay + scale mặt về tư thế chuẩn (mắt nằm ngang)."""
    lx, ly = left_eye
    rx, ry = right_eye
    dx, dy = rx - lx, ry - ly
    if abs(dx) < 1e-3 and abs(dy) < 1e-3:
        return face_bgr
    angle = math.degrees(math.atan2(dy, dx))
    cx = (lx + rx) * 0.5
    cy = (ly + ry) * 0.5
    h, w = face_bgr.shape[:2]
    matrix = cv2.getRotationMatrix2D((cx, cy), angle, 1.0)
    rotated = cv2.warpAffine(
        face_bgr,
        matrix,
        (w, h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REPLICATE,
    )
    eye_dist = max(1.0, math.hypot(dx, dy))
    # Crop vùng mặt: ~2.8× khoảng cách mắt, lấy phần trên (trán→cằm)
    half = eye_dist * 1.4
    top = int(max(0, cy - half * 1.1))
    bottom = int(min(h, cy + half * 1.0))
    left = int(max(0, cx - half))
    right = int(min(w, cx + half))
    if bottom - top < 32 or right - left < 32:
        return rotated
    aligned = rotated[top:bottom, left:right]
    if aligned.size == 0:
        return face_bgr
    return cv2.resize(aligned, (output_size, output_size), interpolation=cv2.INTER_LINEAR)


class FerPlusBackend:
    """FER+ emotion-ferplus-8.onnx — input 64x64 grayscale."""

    INPUT_SIZE = 64

    def __init__(
        self,
        model_path: Optional[str] = None,
        group_margin: float = 0.12,
        use_clahe: bool = True,
        focus_emotion: str = "Angry",
        angry_min_score: float = 0.12,
        angry_margin: float = 0.04,
        angry_neutral_slack: float = 0.08,
        angry_raw_spike: float = 0.04,
    ):
        path = model_path or default_model_path()
        if not os.path.isfile(path):
            raise FileNotFoundError(
                "FER+ ONNX not found: {} — run scripts/download_fer_model.sh".format(path)
            )
        self.model_path = path
        self.group_margin = float(group_margin)
        self.use_clahe = bool(use_clahe)
        self.focus_emotion = str(focus_emotion or "Angry")
        self.angry_min_score = float(angry_min_score)
        self.angry_margin = float(angry_margin)
        self.angry_neutral_slack = float(angry_neutral_slack)
        self.angry_raw_spike = float(angry_raw_spike)
        self.net = cv2.dnn.readNetFromONNX(path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def infer_probs(self, face_bgr: np.ndarray) -> np.ndarray:
        if face_bgr is None or face_bgr.size == 0:
            return np.zeros(len(FERPLUS_LABELS), dtype=np.float64)
        gray = preprocess_face_gray(face_bgr, self.use_clahe)
        gray = cv2.resize(gray, (self.INPUT_SIZE, self.INPUT_SIZE), interpolation=cv2.INTER_LINEAR)
        blob = cv2.dnn.blobFromImage(gray, scalefactor=1.0, size=(self.INPUT_SIZE, self.INPUT_SIZE), mean=0.0)
        self.net.setInput(blob)
        logits = self.net.forward().reshape(-1)
        return softmax(logits)

    def group_scores(self, probs: np.ndarray) -> Dict[str, float]:
        scores = {}
        for social, members in SOCIAL_GROUPS.items():
            if social == "Angry":
                total = 0.0
                for m in members:
                    w = ANGRY_CLASS_WEIGHTS.get(m, 1.0)
                    total += probs[FERPLUS_LABELS.index(m)] * w
                scores[social] = float(total)
            else:
                scores[social] = float(sum(probs[FERPLUS_LABELS.index(m)] for m in members))
        return scores

    def _dominant_angry_raw(self, probs: np.ndarray) -> str:
        angry_members = SOCIAL_GROUPS["Angry"]
        idx = max(
            range(len(angry_members)),
            key=lambda i: probs[FERPLUS_LABELS.index(angry_members[i])]
            * ANGRY_CLASS_WEIGHTS.get(angry_members[i], 1.0),
        )
        return angry_members[idx]

    def predict_from_probs(self, probs: np.ndarray) -> Tuple[str, str, float, Dict[str, float]]:
        """Chọn emotion — ưu tiên Angry khi focus_emotion=Angry (Track D)."""
        group = self.group_scores(probs)
        angry = group.get("Angry", 0.0)
        neutral = group.get("Neutral", 0.0)
        happy = group.get("Happy", 0.0)

        if self.focus_emotion == "Angry":
            p_anger = float(probs[FERPLUS_LABELS.index("anger")])
            p_contempt = float(probs[FERPLUS_LABELS.index("contempt")])
            p_disgust = float(probs[FERPLUS_LABELS.index("disgust")])
            raw_peak = max(p_anger, p_contempt, p_disgust)

            angry_ok = (
                angry >= self.angry_min_score
                and angry + self.angry_neutral_slack >= neutral
                and angry >= happy - self.angry_margin
            )
            # Spike: class giận lẻ đủ mạnh (cau mày, khinh bỉ) dù Neutral vẫn cao
            spike_ok = raw_peak >= self.angry_raw_spike and angry >= happy

            if angry_ok or spike_ok:
                raw = self._dominant_angry_raw(probs)
                return "Angry", raw, float(max(angry, raw_peak)), group

        ranked = sorted(group.items(), key=lambda kv: kv[1], reverse=True)
        best_social, best_score = ranked[0]
        second_score = ranked[1][1] if len(ranked) > 1 else 0.0
        margin = best_score - second_score

        if margin < self.group_margin:
            best_social = "Neutral"
            best_score = neutral

        idx = int(np.argmax(probs))
        raw = FERPLUS_LABELS[idx]
        return best_social, raw, float(best_score), group

    def predict(self, face_bgr: np.ndarray) -> Tuple[str, str, float]:
        probs = self.infer_probs(face_bgr)
        social, raw, conf, _group = self.predict_from_probs(probs)
        return social, raw, conf

    def predict_detailed(self, face_bgr: np.ndarray) -> Tuple[str, str, float, Dict[str, float]]:
        probs = self.infer_probs(face_bgr)
        return self.predict_from_probs(probs)


class YuNetFaceDetector:
    """OpenCV YuNet — chính xác hơn Haar trên webcam."""

    def __init__(self, model_path: Optional[str] = None, score_threshold: float = 0.6):
        path = model_path or default_yunet_path()
        if not os.path.isfile(path):
            raise FileNotFoundError("YuNet not found: {}".format(path))
        self.detector = cv2.FaceDetectorYN.create(path, "", (320, 320), score_threshold, 0.3, 5000)

    def detect(self, image_bgr: np.ndarray) -> list:
        h, w = image_bgr.shape[:2]
        self.detector.setInputSize((w, h))
        _, faces = self.detector.detect(image_bgr)
        if faces is None:
            return []
        # face: x, y, w, h, re_x, re_y, le_x, le_y, ..., score
        return sorted(faces, key=lambda f: f[14] if len(f) > 14 else f[2] * f[3], reverse=True)

    def crop_aligned(self, image_bgr: np.ndarray, face_row: np.ndarray) -> Optional[np.ndarray]:
        x, y, fw, fh = int(face_row[0]), int(face_row[1]), int(face_row[2]), int(face_row[3])
        pad = int(0.2 * max(fw, fh))
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1 = min(image_bgr.shape[1], x + fw + pad)
        y1 = min(image_bgr.shape[0], y + fh + pad)
        crop = image_bgr[y0:y1, x0:x1]
        if crop.size == 0:
            return None
        if len(face_row) >= 15:
            re = (float(face_row[4]), float(face_row[5]))
            le = (float(face_row[6]), float(face_row[7]))
            re = (re[0] - x0, re[1] - y0)
            le = (le[0] - x0, le[1] - y0)
            if re[0] > 0 and le[0] > 0:
                return align_face_by_eyes(crop, le, re)
        return crop


def is_valid_kp2d(keypoints_2d: Sequence[float], idx: int, img_w: int, img_h: int) -> bool:
    if idx * 2 + 1 >= len(keypoints_2d):
        return False
    x = float(keypoints_2d[idx * 2])
    y = float(keypoints_2d[idx * 2 + 1])
    if not (math.isfinite(x) and math.isfinite(y)):
        return False
    if x <= 1.0 and y <= 1.0:
        return False
    if x < 0 or y < 0 or x >= img_w or y >= img_h:
        return False
    return True


def kp2d_xy(keypoints_2d: Sequence[float], idx: int) -> Tuple[float, float]:
    return float(keypoints_2d[idx * 2]), float(keypoints_2d[idx * 2 + 1])


def crop_face_from_body(
    image_bgr: np.ndarray,
    keypoints_2d: Sequence[float],
    body_format: int,
    bbox_tl: Optional[Sequence[float]] = None,
    bbox_br: Optional[Sequence[float]] = None,
    crop_scale: float = 2.2,
    min_crop_px: int = 48,
    align: bool = True,
) -> Optional[np.ndarray]:
    """Crop + căn mặt từ ZED keypoints."""
    h, w = image_bgr.shape[:2]
    if body_format == 34:
        nose_idx, le_idx, re_idx = BODY34_NOSE, BODY34_LEFT_EYE, BODY34_RIGHT_EYE
        head_idx = BODY34_HEAD
    else:
        nose_idx, le_idx, re_idx = BODY18_NOSE, BODY18_LEFT_EYE, BODY18_RIGHT_EYE
        head_idx = BODY18_NOSE

    has_le = is_valid_kp2d(keypoints_2d, le_idx, w, h)
    has_re = is_valid_kp2d(keypoints_2d, re_idx, w, h)
    le = kp2d_xy(keypoints_2d, le_idx) if has_le else None
    re = kp2d_xy(keypoints_2d, re_idx) if has_re else None

    points = []
    for idx in (nose_idx, le_idx, re_idx, head_idx):
        if is_valid_kp2d(keypoints_2d, idx, w, h):
            points.append(kp2d_xy(keypoints_2d, idx))

    if points:
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)
        if has_le and has_re:
            span = max(math.hypot(re[0] - le[0], re[1] - le[1]) * 2.5, float(min_crop_px))
        elif len(points) >= 2:
            span = max(max(xs) - min(xs), max(ys) - min(ys), float(min_crop_px))
        else:
            span = float(min_crop_px)
        half = max(min_crop_px * 0.5, span * crop_scale * 0.5)
    elif bbox_tl is not None and bbox_br is not None:
        x1, y1 = float(bbox_tl[0]), float(bbox_tl[1])
        x2, y2 = float(bbox_br[0]), float(bbox_br[1])
        bw = max(1.0, x2 - x1)
        bh = max(1.0, y2 - y1)
        cx = x1 + bw * 0.5
        cy = y1 + bh * 0.35
        half = max(min_crop_px * 0.5, max(bw, bh) * 0.35)
        le, re = None, None
    else:
        return None

    x0 = int(max(0, math.floor(cx - half)))
    y0 = int(max(0, math.floor(cy - half)))
    x1 = int(min(w, math.ceil(cx + half)))
    y1 = int(min(h, math.ceil(cy + half)))
    if x1 - x0 < min_crop_px or y1 - y0 < min_crop_px:
        return None
    crop = image_bgr[y0:y1, x0:x1]
    if crop.size == 0:
        return None

    if align and le is not None and re is not None:
        le_local = (le[0] - x0, le[1] - y0)
        re_local = (re[0] - x0, re[1] - y0)
        return align_face_by_eyes(crop, le_local, re_local)
    return crop
