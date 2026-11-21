#!/usr/bin/env bash
# run_brain.sh — Launches the full autonomous Z.O.E. hexapod brain stack
# Usage: ./run_brain.sh [extra ros2 launch args]
# Example: ./run_brain.sh model:=nvidia/llama-3.1-nemotron-70b-instruct

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SECRETS_FILE="$SCRIPT_DIR/.brain_secrets"

# ── Load secrets ──────────────────────────────────────────────────────────────
if [[ -f "$SECRETS_FILE" ]]; then
    export $(grep -v '^\s*#' "$SECRETS_FILE" | xargs)
    echo "[brain] Loaded API key from .brain_secrets"
else
    echo "[brain] WARNING: .brain_secrets not found — NVIDIA_API_KEY must be set manually"
fi

if [[ -z "${NVIDIA_API_KEY:-}" ]]; then
    echo "[brain] ERROR: NVIDIA_API_KEY is not set!"
    echo "  Create $SCRIPT_DIR/.brain_secrets with:"
    echo "  NVIDIA_API_KEY=nvapi-xxxx..."
    exit 1
fi

# ── ROS 2 environment (auto-detect Humble or Jazzy) ───────────────────────────
DISTRO="${ROS_DISTRO:-}"
if [[ -z "$DISTRO" ]]; then
    if [[ -f /opt/ros/jazzy/setup.bash ]]; then
        DISTRO="jazzy"
    elif [[ -f /opt/ros/humble/setup.bash ]]; then
        DISTRO="humble"
    else
        echo "[brain] ERROR: No ROS 2 installation found in /opt/ros/"
        exit 1
    fi
fi

source "/opt/ros/${DISTRO}/setup.bash"
source "$SCRIPT_DIR/install/setup.bash"

USER_SITE="$(python3 -c 'import site; print(site.getusersitepackages())')"
export PYTHONPATH="${USER_SITE}:${PYTHONPATH:-}"

echo "[brain] Launching autonomous brain stack (ROS 2 ${DISTRO})..."
echo "[brain] Send queries to: ros2 topic pub --once /brain/query std_msgs/msg/String \"{data: 'Hey Zoe, check your sensors'}\""
echo ""

ros2 launch rover autonomous.launch.py "$@"
