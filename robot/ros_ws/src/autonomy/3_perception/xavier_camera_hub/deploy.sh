#!/usr/bin/env bash
# Copy the camera hub to the Xavier (run on the Thor): ./deploy.sh [user@host]
# Runtime files on the Xavier (state.json, logs, cached SN*.conf) are kept. A running hub
# keeps the old code until it is restarted.
set -euo pipefail
XAVIER=${1:-autolab@192.168.1.101}
DEST=/home/autolab/camera_hub
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

rsync -a --delete --exclude state.json --exclude 'state.json.tmp' --exclude __pycache__ --exclude .pytest_cache \
    --exclude '*.log' --exclude '*.pid' --exclude 'SN*.conf' --exclude 'SN*.conf.tmp' "$HERE/" "$XAVIER:$DEST/"
# The Nano calibration the old server downloaded; copied, not moved (the old service still uses it).
ssh "$XAVIER" "test -f $DEST/SN99292912.conf || cp ~/zedx_nano_stream/SN99292912.conf $DEST/ \
    || echo 'no ~/zedx_nano_stream/SN99292912.conf; the hub will download the Nano calibration'"
echo "deployed to $XAVIER:$DEST"
if ssh "$XAVIER" systemctl is-active --quiet camera-hub.service; then
    echo "camera-hub.service is running the old code: sudo systemctl restart camera-hub.service"
fi
