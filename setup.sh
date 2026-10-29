#!/usr/bin/env bash
set -e

# ============================================================
# Jetson Orin Nano ROS2 Workspace Setup Script
# Supports: --clean-only, --build-only, --rebuild (default)
# ============================================================

# ============================================================
# Examples: ./setup.sh --clean-only
#           ./setup.sh --build-only
#           ./setup.sh --rebuild
#           ./setup.sh --clean-only
# ============================================================
# Features: swap management, workspace cleaning,
#           Workspace sourcing for install/setup.bash (once successfully built), 
#           colcon build, diagnostics
# ============================================================

# Workspace root
WS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "[INFO] Workspace root: $WS_DIR"

# Default mode
MODE="rebuild"

# Parse CLI args
for arg in "$@"; do
  case $arg in
    --clean-only) MODE="clean";;
    --build-only) MODE="build";;
    --rebuild)    MODE="rebuild";;
    *) echo "[WARN] Unknown option: $arg";;
  esac
done
echo "[INFO] Selected mode: $MODE"

# ============================================================
# Ensure swap >= 6GB (important for Jetson builds)
# ============================================================
echo "[INFO] Checking swap space..."
SWAP_MB=$(free -m | awk '/Swap:/ {print $2}')

if [ "$SWAP_MB" -lt 6144 ]; then
  echo "[INFO] Current swap: ${SWAP_MB}MB (<6144MB). Expanding swap..."
  SWAP_FILE="/swapfile"

  # Remove old swap if present
  if [ -f "$SWAP_FILE" ]; then
    echo "[INFO] Removing old swapfile..."
    sudo swapoff "$SWAP_FILE" || true
    sudo rm -f "$SWAP_FILE"
  fi

  # Create 6GB swap
  echo "[INFO] Creating new 6GB swapfile..."
  sudo fallocate -l 6G "$SWAP_FILE" || sudo dd if=/dev/zero of="$SWAP_FILE" bs=1M count=6144
  sudo chmod 600 "$SWAP_FILE"
  sudo mkswap "$SWAP_FILE"
  sudo swapon "$SWAP_FILE"

  # Persist across reboots
  if ! grep -q "$SWAP_FILE" /etc/fstab; then
    echo "[INFO] Adding swapfile entry to /etc/fstab..."
    echo "$SWAP_FILE none swap sw 0 0" | sudo tee -a /etc/fstab
  fi
else
  echo "[INFO] Swap space is sufficient (${SWAP_MB}MB)."
fi

# ============================================================
# Clean (if requested)
# ============================================================
if [ "$MODE" = "clean" ] || [ "$MODE" = "rebuild" ]; then
  echo "[INFO] Cleaning build/, install/, log/"
  rm -rf "$WS_DIR/build" "$WS_DIR/install" "$WS_DIR/log"
fi

# ============================================================
# Source ROS 2 Humble
# ============================================================
if [ -f /opt/ros/humble/setup.bash ]; then
  echo "[INFO] Sourcing ROS 2 Humble..."
  source /opt/ros/humble/setup.bash
else
  echo "[ERROR] ROS 2 Humble not found at /opt/ros/humble"
  exit 1
fi

# ============================================================
# Build (unless clean-only)
# ============================================================
if [ "$MODE" != "clean" ]; then
  echo "[INFO] Starting colcon build..."
  colcon build --symlink-install --parallel-workers $(nproc --ignore=1)
fi

# ============================================================
# Source overlay if build succeeded
# ============================================================
if [ -f "$WS_DIR/install/setup.bash" ]; then
  echo "[INFO] Sourcing overlay: install/setup.bash"
  source "$WS_DIR/install/setup.bash"
else
  if [ "$MODE" != "clean" ]; then
    echo "[ERROR] No install/setup.bash found. Build may have failed."
    exit 1
  fi
fi

# ============================================================
# Diagnostics
# ============================================================
if [ "$MODE" != "clean" ]; then
  echo "[INFO] AMENT_PREFIX_PATH = $AMENT_PREFIX_PATH"
  echo "[INFO] CMAKE_PREFIX_PATH = $CMAKE_PREFIX_PATH"
fi

# Detect WSL vs native
if grep -qi microsoft /proc/version; then
  echo "[INFO] Running inside WSL → skipping Jetson hardware setup."
else
  echo "[INFO] Running on native Jetson/Ubuntu environment."
fi

echo "[SUCCESS] setup.sh completed in mode: $MODE"