#!/usr/bin/env bash
# Tải FER+ ONNX + YuNet vào fer_emotion/models/
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)/models"
MODEL_FILE="${MODEL_DIR}/emotion-ferplus-8.onnx"
URL="https://huggingface.co/onnxmodelzoo/emotion-ferplus-8/resolve/main/emotion-ferplus-8.onnx"
YUNET_URL="https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx"
YUNET_FILE="${MODEL_DIR}/face_detection_yunet_2023mar.onnx"

mkdir -p "${MODEL_DIR}"
if [[ -f "${MODEL_FILE}" ]]; then
  echo "[OK] Model đã có: ${MODEL_FILE} ($(du -h "${MODEL_FILE}" | cut -f1))"
  exit 0
fi
echo "[download] ${URL} → ${MODEL_FILE}"
wget -O "${MODEL_FILE}" "${URL}"
echo "[OK] Done: $(du -h "${MODEL_FILE}" | cut -f1)"

if [[ ! -f "${YUNET_FILE}" ]]; then
  echo "[download] ${YUNET_URL} → ${YUNET_FILE}"
  wget -O "${YUNET_FILE}" "${YUNET_URL}"
fi
echo "[OK] YuNet: $(du -h "${YUNET_FILE}" | cut -f1)"
