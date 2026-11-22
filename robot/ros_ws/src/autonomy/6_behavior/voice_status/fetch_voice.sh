#!/usr/bin/env bash
# Downloads the Piper TTS binary (aarch64) and the female en_US "ljspeech" (high quality) voice into voice/
# (git-ignored). Voices come from the v1.0.0 tag: several on main need a newer Piper than this binary.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p voice
curl -fL https://github.com/rhasspy/piper/releases/download/2023.11.14-2/piper_linux_aarch64.tar.gz | tar -xz -C voice
base=https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_US/ljspeech/high
curl -fL -o voice/en_US-ljspeech-high.onnx "$base/en_US-ljspeech-high.onnx"
curl -fL -o voice/en_US-ljspeech-high.onnx.json "$base/en_US-ljspeech-high.onnx.json"
echo "voice ready: $(pwd)/voice"
