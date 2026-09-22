#!/usr/bin/env bash
# Install and start camera-hub.service (run on the Xavier, from /home/autolab/camera_hub, as a sudoer).
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

if [ "$HERE" != /home/autolab/camera_hub ]; then
    echo "camera-hub.service runs /home/autolab/camera_hub/camera_hub.py; deploy there first (deploy.sh)" >&2
    exit 1
fi
if systemctl is-active --quiet zedx-nano-stream.service; then
    echo "zedx-nano-stream.service is active; it holds port 8090 and the Nano. Stop and disable it first:" >&2
    echo "  sudo systemctl disable --now zedx-nano-stream.service" >&2
    exit 1
fi
if ss -ltnH 'sport = :8090' | grep -q . && ! systemctl is-active --quiet camera-hub.service; then
    echo "port 8090 is in use (a hub started by hand?): $(ss -ltnpH 'sport = :8090' 2>/dev/null)" >&2
    echo "stop it first, e.g. kill \$(cat $HERE/hub.pid)" >&2
    exit 1
fi
sudo install -m 644 "$HERE/camera-hub.service" /etc/systemd/system/camera-hub.service
sudo systemctl daemon-reload
sudo systemctl enable --now camera-hub.service
sleep 2
systemctl --no-pager --lines=5 status camera-hub.service
