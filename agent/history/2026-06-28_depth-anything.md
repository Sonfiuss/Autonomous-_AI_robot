# Session — 2026-06-28 — depth-anything

## What was done

### 1. Requirements & install
- Created `depth-anything/requirements.txt` — Jetson-specific (numpy, matplotlib, Pillow; excludes gradio/demo deps, notes why opencv stays as system package)
- Created `depth-anything/install_jetson.sh` — full install script: apt libopenblas-base → pip packages → NVIDIA torch wheel → torchvision
- Installed via pip: numpy 1.24.4, matplotlib 3.7.5, Pillow 10.4.0
- Installed torch 2.1.0 from NVIDIA JetPack 5 wheel (aarch64, CUDA 11.4):
  `https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl`
- Installed torchvision 0.16.0 from PyPI manylinux aarch64 wheel
- **Pending (needs manual sudo):** `sudo apt-get install -y libopenblas-base` — torch cannot import without this system lib

### 2. Timing log added to `depth_to_3d_annotated.py`
- Added `import time` and helper `_tick(label, t0)` — prints `ms` per step
- Model load is timed once
- Each pipeline step (1–7) timed individually with pixel/point count in label
- Per-image total and grand summary at end
- Output format: `    1. depth inference          3264.8 ms`

### 3. `--infer-scale` flag added
- `parse_args`: `--infer-scale` (float, default 1.0)
- Maps to `infer_size = max(14, int(input_size * infer_scale) // 14 * 14)` — always a multiple of 14 (ViT patch size)
- Passed directly to `model.infer_image(img, infer_size)` — this is the correct knob; pre-scaling the image was ineffective because `Resize(lower_bound)` in `image2tensor` would upscale it back
- CUDA warmup pass added after model load (avoids JIT inflate on first real inference)
- `torch.cuda.synchronize()` added after inference for accurate GPU timing

## Key discoveries
- `image2tensor` uses `Resize(resize_method='lower_bound')` — output shorter side ≥ input_size. Scaling the raw image before passing to `infer_image` had zero effect; only `input_size` (→ `infer_size`) matters.
- First CUDA inference = ~3000ms JIT compile overhead; warmup eliminates this from per-image timing.
- Platform: JetPack R35.6.4 (JetPack 5.1.3), Python 3.8.10, aarch64, CUDA 11.4, device=cuda confirmed.
- torchvision's image.so has an undefined symbol warning (harmless — only affects torchvision.io, not transforms used here).
- xFormers not installed → DINOv2 falls back to standard attention (graceful, just a log warning).

## Files changed
- `depth-anything/requirements.txt` — created
- `depth-anything/install_jetson.sh` — created
- `depth-anything/src/depth_to_3d_annotated.py` — timing + infer-scale

## Pending / next session
- User must run `sudo apt-get install -y libopenblas-base` to make torch importable
- Benchmark actual inference times after warmup at infer-scale 0.25 / 0.5 / 1.0
- Camera config reading (started but interrupted — `/dev/video0` at 1600×1200 @ 30fps via GStreamer backend, only cam 0 available)
