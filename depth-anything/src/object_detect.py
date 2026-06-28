"""
Depth-based obstacle detection dung Depth Anything V2.

Khong can YOLO hay model nhan dang rieng.
Thuat toan:
  1. Depth map -> normalize, invert (0=gan, 1=xa)
  2. Tinh floor depth profile: moi row, lay median cua bottom N% anh lam tham chieu san
  3. Obstacle mask: pixel nao co depth < (floor_ref_tai_row - threshold) -> vat can
  4. Morphological close + findContours -> bounding box
  5. Ve bbox theo zone L/C/R voi depth tuong doi

Vi du:
  python object_detect.py --img ../assets/right_1.jpg
  python object_detect.py --img ../assets/right_1.jpg --gap 0.15 --min-area 800
"""
import os, argparse
import cv2, numpy as np, torch

from depth_anything_v2.dpt import DepthAnythingV2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

CFGS = {
    'vits': {'encoder': 'vits', 'features': 64,  'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
}

ZONE_COLORS = {'L': (0, 200, 60), 'C': (0, 200, 220), 'R': (0, 60, 220)}


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--img',        default=os.path.join(ROOT, 'assets', 'right_1.jpg'))
    p.add_argument('--encoder',    default='vitb', choices=['vits', 'vitb'])
    p.add_argument('--input-size', type=int,   default=518)
    p.add_argument('--device',     default='auto', choices=['auto', 'cuda', 'cpu'])
    p.add_argument('--outdir',     default=os.path.join(ROOT, 'output', 'object_detect'))
    p.add_argument('--blur',       type=int,   default=9,
                   help='Gaussian blur kernel depth. Default=9')
    p.add_argument('--floor-rows', type=float, default=0.25,
                   help='ty le dong duoi dung lam tham chieu san (0..1). Default=0.25')
    p.add_argument('--gap',        type=float, default=0.03,
                   help='nguong gap depth de coi la vat can (0..1). Default=0.03')
    p.add_argument('--gap-scale',  type=float, default=0.4,
                   help='upper anh co gap nho hon: gap_eff = gap * (1 - gap_scale*(1-y/H)). Default=0.4')
    p.add_argument('--min-area',   type=int,   default=300,
                   help='dien tich contour toi thieu (pixel^2). Default=300')
    p.add_argument('--morph',      type=int,   default=15,
                   help='kernel morphological close de lien ket vung. Default=15')
    p.add_argument('--ceil-cut',   type=float, default=0.05,
                   help='cat phan tren cung (ty le) de tranh detect troi/tuong xa. Default=0.05')
    # arm exclusion
    p.add_argument('--arm-region', type=str, default='0.35,0.82,0.65,1.0',
                   help='vung canh tay robot can loai (x1,y1,x2,y2 theo ti le 0..1). Default=0.35,0.82,0.65,1.0')
    p.add_argument('--arm-near',   type=float, default=0.20,
                   help='pixel co depth < gia tri nay trong arm-region se bi bo qua. Default=0.20')
    p.add_argument('--no-arm-mask', action='store_true',
                   help='tat arm exclusion')
    p.add_argument('--no-show',    action='store_true')
    return p.parse_args()


def pick_device(choice):
    if choice == 'cuda':
        return 'cuda' if torch.cuda.is_available() else 'cpu'
    if choice == 'cpu':
        return 'cpu'
    return 'cuda' if torch.cuda.is_available() else 'cpu'


def get_zone(cx, W):
    if cx < W / 3:    return 'L'
    if cx < 2*W / 3:  return 'C'
    return 'R'


def preprocess_depth(depth_raw, blur_k=9):
    d = depth_raw.astype(np.float32)
    d = d.max() - d                                    # invert: 0=gan, 1=xa
    d = (d - d.min()) / (d.max() - d.min() + 1e-8)   # normalize 0..1
    if blur_k > 1:
        k = blur_k if blur_k % 2 == 1 else blur_k + 1
        d = cv2.GaussianBlur(d, (k, k), 0)
    return d


def build_floor_profile(depth_norm, floor_rows=0.25):
    """
    Voi moi row y, uoc tinh depth cua san tai row do.
    Dung bottom floor_rows% anh (goc toa do: 0=tren, H-1=duoi).
    Fit linear depth theo y -> noi suy len cac row phia tren.
    """
    H, W = depth_norm.shape
    n_ref = max(1, int(H * floor_rows))
    ref_region = depth_norm[H - n_ref:, :]           # bottom N rows

    # Median depth moi row trong vung tham chieu
    row_medians = np.median(ref_region, axis=1)       # shape (n_ref,)
    ys = np.arange(H - n_ref, H, dtype=np.float32)

    # Fit bac 1: depth_floor(y) = a*y + b
    coeffs = np.polyfit(ys, row_medians, 1)

    all_ys = np.arange(H, dtype=np.float32)
    floor_profile = np.polyval(coeffs, all_ys)        # shape (H,)
    floor_profile = np.clip(floor_profile, 0.0, 1.0)
    return floor_profile                              # floor_profile[y] = depth san tai row y


def detect_obstacles(depth_norm, floor_profile, gap=0.03, gap_scale=0.4,
                     morph_k=15, min_area=300, ceil_cut=0.05,
                     arm_mask=None, arm_near=0.20):
    """
    Obstacle mask: pixel co depth < (floor_ref - gap_eff) -> gan hon san -> vat can.
    gap_eff giam dan khi len phia tren (anh far = depth contrast nho hon).
    Tra ve list bbox dict {x1,y1,x2,y2,depth_rel,area}.
    """
    H, W = depth_norm.shape

    # gap_eff: giam theo y tu tren xuong (row 0=tren, H-1=duoi)
    # tai row y: gap_eff = gap * (1 - gap_scale * (1 - y/H))
    ys = np.arange(H, dtype=np.float32)
    gap_per_row = gap * (1.0 - gap_scale * (1.0 - ys / H))   # shape (H,)
    gap_per_row = np.clip(gap_per_row, gap * 0.3, gap)        # floor: 30% gap

    floor_map = np.tile(floor_profile[:, None], (1, W))
    gap_map   = np.tile(gap_per_row[:, None],   (1, W))
    obstacle_mask = (depth_norm < (floor_map - gap_map)).astype(np.uint8) * 255

    # Chi cat phan tren cung (troi / tuong rat xa)
    cut = max(1, int(H * ceil_cut))
    obstacle_mask[:cut, :] = 0

    # Loai bo vung canh tay robot: pixel qua gan trong vung co dinh
    if arm_mask is not None:
        ax1, ay1, ax2, ay2 = arm_mask
        px1 = int(ax1 * W); py1 = int(ay1 * H)
        px2 = int(ax2 * W); py2 = int(ay2 * H)
        arm_region_depth = depth_norm[py1:py2, px1:px2]
        # Pixel nao trong vung arm va co depth < arm_near -> xoa khoi mask
        arm_near_mask = (arm_region_depth < arm_near).astype(np.uint8) * 255
        obstacle_mask[py1:py2, px1:px2] = np.where(
            arm_near_mask > 0, 0, obstacle_mask[py1:py2, px1:px2]
        )

    # Morphological close: lien ket cac vung roi rac
    if morph_k > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_k, morph_k))
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(obstacle_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detections = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        x2, y2 = x + w, y + h
        region = depth_norm[y:y2, x:x2]
        depth_rel = float(np.median(region)) if region.size > 0 else 0.0
        detections.append({'x1': x, 'y1': y, 'x2': x2, 'y2': y2,
                           'depth_rel': depth_rel, 'area': int(area)})

    # Sap xep theo area giam dan
    detections.sort(key=lambda d: d['area'], reverse=True)
    return detections, obstacle_mask


def draw_result(img_bgr, detections, depth_norm, obstacle_mask, floor_profile, **kwargs):
    H, W = img_bgr.shape[:2]
    overlay = img_bgr.copy()

    # To mau vung obstacle len overlay (do nhat)
    obs_color = np.zeros_like(overlay)
    obs_color[:] = (30, 30, 200)
    obs_alpha_mask = obstacle_mask > 0
    overlay[obs_alpha_mask] = (
        overlay[obs_alpha_mask] * 0.5 + obs_color[obs_alpha_mask] * 0.5
    ).astype(np.uint8)

    for det in detections:
        x1, y1, x2, y2 = det['x1'], det['y1'], det['x2'], det['y2']
        cx = (x1 + x2) // 2
        zone  = get_zone(cx, W)
        color = ZONE_COLORS[zone]
        depth_pct = int(det['depth_rel'] * 100)

        cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
        txt = f"obstacle d={depth_pct}% [{zone}]"
        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        ly = max(y1 - 4, th + 4)
        cv2.rectangle(overlay, (x1, ly - th - 4), (x1 + tw + 4, ly), color, -1)
        cv2.putText(overlay, txt, (x1 + 2, ly - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

    # Ve floor profile line
    for y in range(0, H, 4):
        fx = int(floor_profile[y] * (W - 1))
        cv2.circle(overlay, (fx, y), 1, (0, 255, 255), -1)

    # Ve vung arm exclusion (tim nhat)
    if 'arm_mask' in kwargs and kwargs['arm_mask'] is not None:
        ax1, ay1, ax2, ay2 = kwargs['arm_mask']
        px1,py1,px2,py2 = int(ax1*W),int(ay1*H),int(ax2*W),int(ay2*H)
        cv2.rectangle(overlay, (px1,py1), (px2,py2), (200,0,200), 1)
        cv2.putText(overlay, 'ARM', (px1+2, py1+14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200,0,200), 1)

    # Zone dividers
    cv2.line(overlay, (W // 3, 0), (W // 3, H), (180, 180, 180), 1)
    cv2.line(overlay, (2*W // 3, 0), (2*W // 3, H), (180, 180, 180), 1)

    # Zone header
    zone_w = W // 3
    zones_det = {'L': 0, 'C': 0, 'R': 0}
    for det in detections:
        zones_det[get_zone((det['x1']+det['x2'])//2, W)] += 1
    for i, (z, x0) in enumerate(zip(('L', 'C', 'R'), (0, zone_w, 2*zone_w))):
        n = zones_det[z]
        status = 'BLOCKED' if n > 0 else 'CLEAR'
        col = (0, 60, 220) if n > 0 else (0, 200, 60)
        cv2.rectangle(overlay, (x0+2, 4), (x0+zone_w-2, 30), col, -1)
        cv2.putText(overlay, f'{z}:{status}({n})', (x0+5, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    # Depth colormap panel
    d8 = (depth_norm * 255).astype(np.uint8)
    depth_vis = cv2.applyColorMap(d8, cv2.COLORMAP_INFERNO)

    # Obstacle mask panel (grayscale -> color)
    mask_vis = cv2.cvtColor(obstacle_mask, cv2.COLOR_GRAY2BGR)

    combined = np.hstack([img_bgr, depth_vis, mask_vis, overlay])
    return combined


def main():
    args = parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    device = pick_device(args.device)
    print(f'device={device}  encoder={args.encoder}')

    model = DepthAnythingV2(**CFGS[args.encoder])
    ckpt = os.path.join(ROOT, 'model', f'depth_anything_v2_{args.encoder}.pth')
    model.load_state_dict(torch.load(ckpt, map_location='cpu', weights_only=True))
    model = model.to(device).eval()

    img = cv2.imread(args.img)
    if img is None:
        raise FileNotFoundError(f'Khong doc duoc: {args.img}')
    H, W = img.shape[:2]

    print('Depth inference...')
    depth_raw = model.infer_image(img, args.input_size)
    depth_raw = cv2.resize(depth_raw.astype(np.float32), (W, H), interpolation=cv2.INTER_LINEAR)
    depth_norm = preprocess_depth(depth_raw, args.blur)

    floor_profile = build_floor_profile(depth_norm, args.floor_rows)
    # Arm exclusion mask
    arm_mask = None
    if not args.no_arm_mask:
        try:
            ax1, ay1, ax2, ay2 = [float(v) for v in args.arm_region.split(',')]
            arm_mask = (ax1, ay1, ax2, ay2)
        except ValueError:
            print(f'[WARN] --arm-region khong hop le: {args.arm_region}')

    detections, obstacle_mask = detect_obstacles(
        depth_norm, floor_profile,
        gap=args.gap, gap_scale=args.gap_scale,
        morph_k=args.morph, min_area=args.min_area, ceil_cut=args.ceil_cut,
        arm_mask=arm_mask, arm_near=args.arm_near,
    )

    print(f'Tim thay {len(detections)} vung vat can.')
    zones = {'L': [], 'C': [], 'R': []}
    for det in detections:
        zones[get_zone((det['x1']+det['x2'])//2, W)].append(det)
    for z in ('L', 'C', 'R'):
        items = ', '.join(f"d={int(d['depth_rel']*100)}%" for d in zones[z]) or '-'
        print(f'  {z}: {items}')

    combined = draw_result(img, detections, depth_norm, obstacle_mask, floor_profile,
                           arm_mask=arm_mask)

    stem = os.path.splitext(os.path.basename(args.img))[0]
    out_path = os.path.join(args.outdir, f'{stem}_det.jpg')
    cv2.imwrite(out_path, combined)
    print(f'Saved: {out_path}')

    if not args.no_show:
        scale = min(1.0, 1800 / combined.shape[1])
        show = cv2.resize(combined, (0, 0), fx=scale, fy=scale)
        cv2.imshow('Obstacle Detect  [q=thoat]', show)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
