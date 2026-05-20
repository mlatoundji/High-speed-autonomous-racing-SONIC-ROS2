#!/usr/bin/env bash
set -eo pipefail

export DISPLAY="${DISPLAY:-:1}"
export VNC_RESOLUTION="${VNC_RESOLUTION:-1280x800}"
export AUTOCAR_WORKSPACE="${AUTOCAR_WORKSPACE:-/workspace}"
export AUTOCAR_SHARED_DIR="${AUTOCAR_SHARED_DIR:-/workspace/shared}"
export AUTOCAR_LOG_DIR="${AUTOCAR_LOG_DIR:-/workspace/runtime/logs}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"

mkdir -p "${AUTOCAR_SHARED_DIR}" "${AUTOCAR_LOG_DIR}" "${XDG_RUNTIME_DIR}"
chmod 700 "${XDG_RUNTIME_DIR}"

cleanup() {
  kill "${api_pid:-0}" "${novnc_pid:-0}" "${x11vnc_pid:-0}" "${xfce_pid:-0}" "${xvfb_pid:-0}" 2>/dev/null || true
}
trap cleanup TERM INT EXIT

Xvfb "${DISPLAY}" -screen 0 "${VNC_RESOLUTION}x24" -ac +extension GLX +render -noreset \
  >"${AUTOCAR_LOG_DIR}/xvfb.log" 2>&1 &
xvfb_pid=$!

sleep 1

dbus-launch --exit-with-session startxfce4 \
  >"${AUTOCAR_LOG_DIR}/xfce.log" 2>&1 &
xfce_pid=$!

x11vnc -display "${DISPLAY}" -forever -shared -nopw -listen 0.0.0.0 -xkb \
  >"${AUTOCAR_LOG_DIR}/x11vnc.log" 2>&1 &
x11vnc_pid=$!

websockify --web=/usr/share/novnc 0.0.0.0:6080 localhost:5900 \
  >"${AUTOCAR_LOG_DIR}/novnc.log" 2>&1 &
novnc_pid=$!

source /opt/ros/foxy/setup.bash
source "${AUTOCAR_WORKSPACE}/install/setup.bash"

uvicorn api.control_api:app --host 0.0.0.0 --port "${API_PORT:-8001}" \
  >"${AUTOCAR_LOG_DIR}/api.log" 2>&1 &
api_pid=$!

wait "${api_pid}"
