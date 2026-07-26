---
id: 2026-07-16_yolo-object-mask
status: testing
module: stereo-camera
started: 2026-07-16
---

## Task
YOLO-seg protect mask so flatten-planes stops compressing floor obstacles into the floor/wall planes.

## Input
- Problem: `--flatten-planes` flattens walls correctly but also squashes low obstacles (anything within band 0.12m of floor / 0.12-0.30m of wall) — objects lose solid form.
- User direction: use YOLO to detect objects; detected object pixels must survive flattening.
- Env: ultralytics 8.4.80 already installed on D:\University\Python\Python310\python.exe (torch OK).

## Expected output
`stereo_cloud.py --flatten-planes --yolo` on a pair with floor obstacles: walls/floor still flatten, detected objects keep full DA-V2 shape. Debug `yolo_overlay.jpg` + `objects.json` + holes in `flatten_labels.jpg`.

## Plan
- [x] 1. YOLO-seg helper: run yolo11n-seg on raw right color image, union instance masks, dilate, remap to rectified frame via calib.map_r. Flags --yolo / --yolo-conf / --yolo-model (off by default).
- [x] 2. Protect mask into flatten_planes(): protected pixels excluded from capture() `near` mask (never pulled onto floor/wall planes).
- [x] 3. Drop golden anchors inside object masks from wall line-RANSAC (no phantom wall from a big object face).
- [x] 4. Debug outputs: yolo_overlay.jpg (masks+boxes on rectified right), objects.json (class, conf, bbox, median Z, 3D centroid).
- [x] 5. Test on pair with obstacles near wall; compare object form + wall rms before/after.

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
- [--:--] step 1: yolo_detect() added — masks.xy polygons (letterbox already undone), fillPoly, dilate, remap via map_r | info: bbox fallback if detect-only ckpt
- [--:--] step 2: protect excluded via `good &= ~protect` in flatten_planes — covers floor fit, floor capture, wall capture in one gate
- [--:--] step 3: anchors on protect mask dropped before wall line-RANSAC
- [--:--] step 4: objects.json (class/conf/bbox/z_median/centroid from FINAL Z) + yolo_overlay.jpg on rectified right
- [--:--] step 5: A/B on _180 (--disp-offset 0): yolo found cup/bowl/bottle/bowl (bottle 1.91m vs tape 1.92m), 40k px protected, 22 anchors dropped | info: floor capture 177k->158k px, wall2 104k->94k px, plane normals unchanged to 0.01 -> protection does not degrade fits
- [--:--] misc: yolo11n-seg.pt (5.9MB) auto-downloaded, moved to depth-anything/model/; model path resolved from model/ when bare name given | info: COCO misses boots/drill/slippers on this frame as predicted
- [--:--] step 4b (user request): debug_dir in yolo_detect -> yolo_raw_detect.jpg (res.plot() on RAW image) + yolo_protect_mask.jpg (union, rectified) | info: blue container double-detected as cup 0.36 + bowl 0.32 (same object, harmless for protection)

## Test result
<!-- Filled when user confirms. Pass / Fail / Partial + brief note -->
