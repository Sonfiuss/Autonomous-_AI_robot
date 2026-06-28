#!/usr/bin/env bash
# Install script for Depth-Anything V2 on NVIDIA Jetson (JetPack 5.x)
# Tested on: JetPack 5.1.x (R35), Python 3.8, aarch64, CUDA 11.4
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY=python3
PIP=pip3

echo "=== System info ==="
$PY --version
uname -m
cat /etc/nv_tegra_release 2>/dev/null || true

# ── Step 0: system dependency (libopenblas) ───────────────────────────────────
echo ""
echo "=== System packages ==="
sudo apt-get install -y libopenblas-base

# ── Step 1: pip packages ──────────────────────────────────────────────────────
echo ""
echo "=== Installing pip packages ==="
$PIP install -r "$SCRIPT_DIR/requirements.txt"

# ── Step 2: PyTorch (NVIDIA JetPack 5 wheel) ──────────────────────────────────
echo ""
echo "=== PyTorch ==="
if $PY -c "import torch; print('torch', torch.__version__)" 2>/dev/null; then
    echo "  already installed, skipping."
else
    # NVIDIA provides JetPack-specific wheels at:
    # https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/
    # For JetPack 5.1.x + Python 3.8 + CUDA 11.4:
    TORCH_WHL="torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl"
    TORCH_URL="https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/${TORCH_WHL}"
    TMP_WHL="/tmp/${TORCH_WHL}"

    echo "  Downloading ${TORCH_WHL} ..."
    wget -q --show-progress "${TORCH_URL}" -O "${TMP_WHL}"
    $PIP install "${TMP_WHL}"
    rm -f "${TMP_WHL}"
fi

# ── Step 3: torchvision (NVIDIA JetPack 5 wheel) ──────────────────────────────
echo ""
echo "=== torchvision ==="
if $PY -c "import torchvision; print('torchvision', torchvision.__version__)" 2>/dev/null; then
    echo "  already installed, skipping."
else
    TV_WHL="torchvision-0.16.2a0+cbb34ae3-cp38-cp38-linux_aarch64.whl"
    TV_URL="https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/${TV_WHL}"
    TMP_TV="/tmp/${TV_WHL}"

    echo "  Downloading ${TV_WHL} ..."
    if wget -q --show-progress "${TV_URL}" -O "${TMP_TV}" 2>/dev/null; then
        $PIP install "${TMP_TV}"
        rm -f "${TMP_TV}"
    else
        echo "  NVIDIA wheel not found at default URL."
        echo "  Trying PyPI build for aarch64 ..."
        # Fallback: build from source or use NVIDIA's container
        $PIP install "torchvision==0.16.2" --no-deps || {
            echo ""
            echo "  torchvision install failed. Manual options:"
            echo "  a) Use NVIDIA L4T PyTorch Docker image (recommended for prod)"
            echo "     https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-pytorch"
            echo "  b) Build from source:"
            echo "     git clone https://github.com/pytorch/vision torchvision"
            echo "     cd torchvision && git checkout v0.16.2"
            echo "     python3 setup.py install --user"
        }
    fi
fi

# ── Step 4: Verify ────────────────────────────────────────────────────────────
echo ""
echo "=== Verification ==="
$PY - <<'EOF'
import sys
ok = True

def check(name, fn):
    global ok
    try:
        result = fn()
        print(f"  [OK] {name}: {result}")
    except Exception as e:
        print(f"  [FAIL] {name}: {e}")
        ok = False

check("numpy",      lambda: __import__("numpy").__version__)
check("matplotlib", lambda: __import__("matplotlib").__version__)
check("Pillow",     lambda: __import__("PIL").__version__)
check("cv2",        lambda: __import__("cv2").__version__)

try:
    import torch
    v = torch.__version__
    cuda = torch.cuda.is_available()
    gpu = torch.cuda.get_device_name(0) if cuda else "N/A"
    print(f"  [OK] torch: {v}  cuda={cuda}  gpu={gpu}")
except Exception as e:
    print(f"  [FAIL] torch: {e}")
    ok = False

try:
    import torchvision
    print(f"  [OK] torchvision: {torchvision.__version__}")
except Exception as e:
    print(f"  [FAIL] torchvision: {e}")
    ok = False

sys.exit(0 if ok else 1)
EOF

echo ""
echo "=== Done ==="
