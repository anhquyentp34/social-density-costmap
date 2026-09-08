#!/usr/bin/env bash
# Tải FER+ ONNX (34 MB) nếu chưa có — dùng cho emotion_detection_node.py
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)/models"
MODEL_FILE="${MODEL_DIR}/emotion-ferplus-8.onnx"
URL="https://huggingface.co/onnxmodelzoo/emotion-ferplus-8/resolve/main/emotion-ferplus-8.onnx"

mkdir -p "${MODEL_DIR}"
if [[ -f "${MODEL_FILE}" ]]; then
  echo "[OK] Model đã có: ${MODEL_FILE} ($(du -h "${MODEL_FILE}" | cut -f1))"
  exit 0
fi
echo "[download] ${URL} → ${MODEL_FILE}"
wget -O "${MODEL_FILE}" "${URL}"
echo "[OK] Done: $(du -h "${MODEL_FILE}" | cut -f1)"
