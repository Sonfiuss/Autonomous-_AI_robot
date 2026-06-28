"""
Xac dinh vung co the di chuyen bang ray-linearity analysis.

Tu diem bottom-center, ban cac ray theo nhieu huong (moi 10 do).
Tren moi ray, depth tang tuyen tinh = san; khi tinh tuyen tinh pha vo = ranh gioi vat can.
Polygon noi cac ranh gioi = drive area, to mau xanh la.

Vi du:
  python drive_area.py --img ../assets/right_1.jpg
  python drive_area.py --img ../assets/right_1.jpg --r2-min 0.75 --ray-step 4
"""
import os, argparse
import cv2, numpy as np, torch

from depth_anything_v2.dpt import DepthAnythingV2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

CFGS = {
    'vits': {'encoder': 'vits', 'features': 64,  'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
}


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--img', default=os.path.join(ROOT, 'assets', 'right_1.jpg'))
    p.add_argument('--encoder', default='vitb', choices=['vits', 'vitb'])
    p.add_argument('--input-size', type=int, default=518)
    p.add_argument('--device', default='auto', choices=['auto', 'cuda', 'cpu'])
    p.add_argument('--outdir', default=os.path.join(ROOT, 'output', 'drive_area'))
    # ray params
    p.add_argument('--n-rays', type=int, default=40,
                   help='so ray doc theo canh duoi. Default=40')
    p.add_argument('--ray-step', type=int, default=4,
                   help='buoc lay mau tren ray (pixel). Default=4')
    p.add_argument('--min-lin', type=int, default=10,
                   help='so diem toi thieu de xac lap mo hinh tuyen tinh. Default=10')
    p.add_argument('--r2-min', type=float, default=0.999,
                   help='nguong R^2 de coi la tuyen tinh (0..1). Default=0.92')
    p.add_argument('--blur', type=int, default=15,
                   help='kernel blur depth truoc khi phan tich (le). Default=15')
    p.add_argument('--alpha', type=float, default=0.50,
                   help='do trong suot overlay xanh. Default=0.50')
    return p.parse_args()


def pick_device(choice):
    if choice == 'cuda':
        return 'cuda' if torch.cuda.is_available() else 'cpu'
    if choice == 'cpu':
        return 'cpu'
    return 'cuda' if torch.cuda.is_available() else 'cpu'


def preprocess_depth(depth_raw, blur_k=15):
    """Invert, normalize 0..1, blur nhe de giam nhieu."""
    d = depth_raw.astype(np.float32)
    d = d.max() - d                                   # invert: lon = xa
    d = (d - d.min()) / (d.max() - d.min() + 1e-8)  # normalize 0..1
    if blur_k > 1:
        k = blur_k if blur_k % 2 == 1 else blur_k + 1
        d = cv2.GaussianBlur(d, (k, k), 0)
    return d


def cast_ray_to_top(depth_norm, bx, ray_step=4, top_cx=None):
    """
    Ban ray tu (bx, H-1) tren canh duoi huong ve trung diem canh tren (top_cx, 0).
    Tra ve: list (y, x, depth, dist_pixel)
    """
    H, W = depth_norm.shape
    if top_cx is None:
        top_cx = W // 2

    # Vector huong: tu (bx, H-1) -> (top_cx, 0)
    vx = float(top_cx - bx)
    vy = float(0 - (H - 1))          # am = len tren
    length = np.hypot(vx, vy)
    dx, dy = vx / length, vy / length

    samples = []
    t = ray_step
    while True:
        x = int(round(bx + dx * t))
        y = int(round((H - 1) + dy * t))
        if not (0 <= x < W and 0 <= y < H):
            break
        samples.append((y, x, float(depth_norm[y, x]), float(t)))
        t += ray_step
    return samples


def find_floor_boundary(samples, min_lin=10, r2_min=0.92):
    """
    Tren 1 ray, tim chi so boundary: diem cuoi cung con la san (depth tuyen tinh).
    Mo rong dan tu min_lin diem; khi R^2 < r2_min thi dung lai.
    """
    n = len(samples)
    if n < min_lin:
        return n - 1

    depths = np.array([s[2] for s in samples])
    dists  = np.array([s[3] for s in samples])

    boundary = n - 1

    for end in range(min_lin + 1, n + 1):
        d_sub = depths[:end]
        t_sub = dists[:end]
        coeffs = np.polyfit(t_sub, d_sub, 1)
        d_pred = np.polyval(coeffs, t_sub)
        ss_res = np.sum((d_sub - d_pred) ** 2)
        ss_tot = np.sum((d_sub - d_sub.mean()) ** 2)
        r2     = 1.0 - ss_res / (ss_tot + 1e-10)

        if r2 < r2_min:
            boundary = end - 2
            break

    return max(0, boundary)


def compute_drive_polygon(depth_norm, ray_step=4, min_lin=10, r2_min=0.92, n_rays=40):
    """
    Chay n_rays ray tu canh duoi -> trung diem canh tren.
    Tra ve polygon va debug_rays.
    """
    H, W = depth_norm.shape
    top_cx = W // 2

    # Cac diem xuat phat tren canh duoi, phan bo deu
    origins_x = np.linspace(0, W - 1, n_rays, dtype=int)

    boundary_pts = []
    debug_rays   = []

    for bx in origins_x:
        samples = cast_ray_to_top(depth_norm, int(bx), ray_step, top_cx)
        if not samples:
            continue
        bi = find_floor_boundary(samples, min_lin, r2_min)
        by, bx_b = samples[bi][0], samples[bi][1]
        boundary_pts.append((bx_b, by))
        debug_rays.append({'samples': samples, 'boundary_idx': bi, 'origin_x': int(bx)})

    # Polygon: canh duoi trai -> boundary (trai->phai) -> canh duoi phai
    polygon = [(0, H - 1)] + boundary_pts + [(W - 1, H - 1)]
    return np.array(polygon, dtype=np.int32), debug_rays


def draw_result(img_bgr, polygon, debug_rays, depth_norm, alpha=0.50):
    H, W = img_bgr.shape[:2]
    overlay = img_bgr.copy()

    # Fill drive area xanh la
    mask = np.zeros((H, W), dtype=np.uint8)
    cv2.fillPoly(mask, [polygon], 255)
    green = np.zeros_like(img_bgr)
    green[:] = (30, 200, 60)
    overlay[mask == 255] = (
        overlay[mask == 255] * (1 - alpha) + green[mask == 255] * alpha
    ).astype(np.uint8)

    # Ve duong bien drive area
    cv2.polylines(overlay, [polygon], isClosed=False, color=(0, 255, 0), thickness=2)

    # Ve cac ray (mau gradient xanh -> vang -> do theo ty le boundary)
    for ray in debug_rays:
        samples = ray['samples']
        bi      = ray['boundary_idx']
        for i, (sy, sx, _, _) in enumerate(samples):
            if i < bi:
                color = (0, 180, 0)    # san: xanh
            elif i == bi:
                color = (0, 255, 255)  # ranh gioi: vang
            else:
                color = (0, 60, 200)   # vat can: do
            cv2.circle(overlay, (sx, sy), 1, color, -1)

    # Zone L/C/R
    zone_w = W // 3
    zone_labels = [('L', 0), ('C', zone_w), ('R', 2 * zone_w)]
    for label, x0 in zone_labels:
        x1 = x0 + zone_w
        region_mask = mask[:, x0:x1]
        free_ratio = (region_mask == 255).mean()
        status = 'CLEAR' if free_ratio > 0.10 else 'BLOCKED'
        col = (0, 210, 0) if status == 'CLEAR' else (0, 0, 210)
        cv2.rectangle(overlay, (x0 + 2, 4), (x1 - 2, 34), col, -1)
        cv2.rectangle(overlay, (x0 + 2, 4), (x1 - 2, 34), (0, 0, 0), 1)
        cv2.putText(overlay, f'{label}:{status}', (x0 + 5, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 2)

    # Ve diem hoi tu tren (trung diem canh tren)
    cv2.circle(overlay, (W // 2, 0), 6, (255, 255, 0), -1)

# Depth colormap de kiem tra
    d8 = (depth_norm * 255).astype(np.uint8)
    depth_vis = cv2.applyColorMap(d8, cv2.COLORMAP_INFERNO)
    cv2.polylines(depth_vis, [polygon], isClosed=False, color=(0, 255, 0), thickness=2)

    combined = np.hstack([img_bgr, depth_vis, overlay])
    return combined, mask


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
    depth_raw = cv2.resize(depth_raw.astype(np.float32), (W, H),
                           interpolation=cv2.INTER_LINEAR)
    depth_norm = preprocess_depth(depth_raw, args.blur)

    polygon, debug_rays = compute_drive_polygon(
        depth_norm,
        ray_step=args.ray_step, min_lin=args.min_lin,
        r2_min=args.r2_min, n_rays=args.n_rays,
    )

    combined, _ = draw_result(img, polygon, debug_rays, depth_norm, args.alpha)

    stem = os.path.splitext(os.path.basename(args.img))[0]
    out_path = os.path.join(args.outdir, f'{stem}_drive.jpg')
    cv2.imwrite(out_path, combined)
    print(f'Saved: {out_path}')

    scale = min(1.0, 1600 / combined.shape[1])
    show = cv2.resize(combined, (0, 0), fx=scale, fy=scale)
    cv2.imshow('Drive Area  [q=thoat]', show)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
