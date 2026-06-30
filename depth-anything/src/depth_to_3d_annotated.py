"""
Fuse depth + obstacle detection + drive area -> annotated point cloud (.ply).

Moi diem 3D duoc to mau theo nhan semantic:
  - Do   (255, 60,  60 ) = obstacle (tu object_detect)
  - Xanh (60,  220, 60 ) = drive area (tu drive_area)
  - Vang (255, 220, 60 ) = overlap (obstacle trong drive area - uu tien obstacle)
  - Goc                  = khong xac dinh

Vi du:
  python depth_to_3d_annotated.py --img ../assets/right_1.jpg
  python depth_to_3d_annotated.py --img ../assets --stride 2 --show
"""
import os, sys, glob, argparse, time
import cv2, numpy as np, torch

from depth_anything_v2.dpt import DepthAnythingV2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

CFGS = {
    'vits': {'encoder': 'vits', 'features': 64,  'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
}

COLOR_OBSTACLE  = np.array([255,  60,  60], dtype=np.uint8)   # do
COLOR_FREE      = np.array([ 60, 220,  60], dtype=np.uint8)   # xanh la
COLOR_OVERLAP   = np.array([255, 220,  60], dtype=np.uint8)   # vang (obstacle uu tien)


# ---------------------------------------------------------------------------
# Args
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description='Fused depth + obstacle + drive area -> PLY')
    p.add_argument('--img',        default=os.path.join(ROOT, 'assets'),
                   help='file anh hoac thu muc')
    p.add_argument('--encoder',    default='vits', choices=['vits', 'vitb'])
    p.add_argument('--input-size', type=int,   default=518)
    p.add_argument('--infer-scale', type=float, default=1.0,
                   help='scale anh truoc inference (0.5 = giam 1/2, nhanh hon ~4x)')
    p.add_argument('--device',     default='auto', choices=['auto', 'cuda', 'cpu'])
    p.add_argument('--outdir',     default=os.path.join(ROOT, 'output', 'pointcloud_annotated'))
    p.add_argument('--fov',        type=float, default=60.0)
    p.add_argument('--stride',     type=int,   default=2)
    p.add_argument('--camera-height', type=float, default=0.32)
    p.add_argument('--tilt',       type=float, default=15.0)
    p.add_argument('--floor-rows', type=float, default=0.25)
    p.add_argument('--no-flatten', action='store_true')
    # obstacle params
    p.add_argument('--blur',       type=int,   default=9)
    p.add_argument('--gap',        type=float, default=0.03)
    p.add_argument('--gap-scale',  type=float, default=0.4)
    p.add_argument('--min-area',   type=int,   default=300)
    p.add_argument('--morph',      type=int,   default=15)
    p.add_argument('--ceil-cut',   type=float, default=0.05)
    p.add_argument('--arm-region', type=str,   default='0.35,0.82,0.65,1.0')
    p.add_argument('--arm-near',   type=float, default=0.20)
    p.add_argument('--no-arm-mask', action='store_true')
    # drive area params
    p.add_argument('--n-rays',     type=int,   default=40)
    p.add_argument('--ray-step',   type=int,   default=4)
    p.add_argument('--min-lin',    type=int,   default=10)
    p.add_argument('--r2-min',     type=float, default=0.993)
    p.add_argument('--drive-blur', type=int,   default=15)
    # misc
    p.add_argument('--show', action='store_true', help='mo cua so 3D (can open3d)')
    return p.parse_args()


def _tick(label, t0):
    elapsed = time.time() - t0
    print(f'    {label:<26s} {elapsed*1000:7.1f} ms')
    return elapsed


def pick_device(choice):
    if choice == 'cuda':
        if not torch.cuda.is_available():
            raise RuntimeError('--device cuda nhung torch la ban CPU-only.')
        return 'cuda'
    if choice == 'cpu':
        return 'cpu'
    return 'cuda' if torch.cuda.is_available() else 'cpu'


# ---------------------------------------------------------------------------
# Depth preprocessing (dung chung cho ca obstacle va drive area)
# ---------------------------------------------------------------------------

def preprocess_depth(depth_raw, blur_k=9):
    d = depth_raw.astype(np.float32)
    d = d.max() - d
    d = (d - d.min()) / (d.max() - d.min() + 1e-8)
    if blur_k > 1:
        k = blur_k if blur_k % 2 == 1 else blur_k + 1
        d = cv2.GaussianBlur(d, (k, k), 0)
    return d


# ---------------------------------------------------------------------------
# Obstacle mask (tu object_detect.py)
# ---------------------------------------------------------------------------

def build_floor_profile(depth_norm, floor_rows=0.25):
    H, W = depth_norm.shape
    n_ref = max(1, int(H * floor_rows))
    ref_region = depth_norm[H - n_ref:, :]
    row_medians = np.median(ref_region, axis=1)
    ys = np.arange(H - n_ref, H, dtype=np.float32)
    coeffs = np.polyfit(ys, row_medians, 1)
    all_ys = np.arange(H, dtype=np.float32)
    floor_profile = np.clip(np.polyval(coeffs, all_ys), 0.0, 1.0)
    return floor_profile


def compute_obstacle_mask(depth_norm, floor_profile, gap=0.03, gap_scale=0.4,
                          morph_k=15, min_area=300, ceil_cut=0.05,
                          arm_mask=None, arm_near=0.20):
    H, W = depth_norm.shape
    ys = np.arange(H, dtype=np.float32)
    gap_per_row = gap * (1.0 - gap_scale * (1.0 - ys / H))
    gap_per_row = np.clip(gap_per_row, gap * 0.3, gap)

    floor_map = np.tile(floor_profile[:, None], (1, W))
    gap_map   = np.tile(gap_per_row[:, None],   (1, W))
    mask = (depth_norm < (floor_map - gap_map)).astype(np.uint8) * 255

    cut = max(1, int(H * ceil_cut))
    mask[:cut, :] = 0

    if arm_mask is not None:
        ax1, ay1, ax2, ay2 = arm_mask
        px1, py1 = int(ax1 * W), int(ay1 * H)
        px2, py2 = int(ax2 * W), int(ay2 * H)
        near = (depth_norm[py1:py2, px1:px2] < arm_near).astype(np.uint8) * 255
        mask[py1:py2, px1:px2] = np.where(near > 0, 0, mask[py1:py2, px1:px2])

    if morph_k > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_k, morph_k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Loc contour nho
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filtered = np.zeros_like(mask)
    for cnt in contours:
        if cv2.contourArea(cnt) >= min_area:
            cv2.drawContours(filtered, [cnt], -1, 255, -1)

    return filtered  # uint8 mask: 255=obstacle


# ---------------------------------------------------------------------------
# Drive area mask (tu drive_area.py)
# ---------------------------------------------------------------------------

def cast_ray(depth_norm, bx, ray_step=4):
    H, W = depth_norm.shape
    top_cx = W // 2
    vx = float(top_cx - bx)
    vy = float(-(H - 1))
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


def find_boundary(samples, min_lin=10, r2_min=0.993):
    n = len(samples)
    if n < min_lin:
        return n - 1
    depths = np.array([s[2] for s in samples])
    dists  = np.array([s[3] for s in samples])
    boundary = n - 1
    for end in range(min_lin + 1, n + 1):
        coeffs = np.polyfit(dists[:end], depths[:end], 1)
        pred   = np.polyval(coeffs, dists[:end])
        ss_res = np.sum((depths[:end] - pred) ** 2)
        ss_tot = np.sum((depths[:end] - depths[:end].mean()) ** 2)
        r2 = 1.0 - ss_res / (ss_tot + 1e-10)
        if r2 < r2_min:
            boundary = end - 2
            break
    return max(0, boundary)


def compute_drive_mask(depth_norm, n_rays=40, ray_step=4, min_lin=10, r2_min=0.993):
    H, W = depth_norm.shape
    origins_x = np.linspace(0, W - 1, n_rays, dtype=int)
    boundary_pts = []
    for bx in origins_x:
        samples = cast_ray(depth_norm, int(bx), ray_step)
        if not samples:
            continue
        bi = find_boundary(samples, min_lin, r2_min)
        boundary_pts.append((samples[bi][1], samples[bi][0]))

    polygon = np.array([(0, H - 1)] + boundary_pts + [(W - 1, H - 1)], dtype=np.int32)
    mask = np.zeros((H, W), dtype=np.uint8)
    cv2.fillPoly(mask, [polygon], 255)
    return mask  # uint8 mask: 255=drivable


# ---------------------------------------------------------------------------
# 3D projection (tu depth_to_3d.py)
# ---------------------------------------------------------------------------

def depth_to_points(depth_raw, color_bgr, fov_deg, stride, camera_height, tilt_deg):
    H, W = depth_raw.shape
    fx = fy = (W / 2.0) / np.tan(np.deg2rad(fov_deg) / 2.0)
    cx, cy = W / 2.0, H / 2.0

    d = depth_raw.astype(np.float32)
    d = d.max() - d
    d = d / (d.max() + 1e-8) * 5.0 + 0.5

    ys, xs = np.mgrid[0:H:stride, 0:W:stride]
    z = d[ys, xs]

    Xc = (xs - cx) * z / fx
    Yc = -(ys - cy) * z / fy
    Zc = z

    t = np.deg2rad(tilt_deg)
    Xw =  Xc
    Yw =  Yc * np.cos(t) + Zc * np.sin(t)
    Zw = -Yc * np.sin(t) + Zc * np.cos(t)

    pts  = np.stack([Xw, Yw, Zw], axis=-1).reshape(-1, 3)
    pts[:, 1] += camera_height

    # Lay mau RGB goc
    cols = color_bgr[ys, xs][:, :, ::-1].reshape(-1, 3).astype(np.uint8)

    # Lay mau mask de map semantic
    return pts, cols, ys.reshape(-1), xs.reshape(-1)


def fit_and_flatten(pts, floor_mask_flat):
    floor_pts = pts[floor_mask_flat]
    if len(floor_pts) < 50:
        return pts
    if len(floor_pts) > 2000:
        idx = np.random.choice(len(floor_pts), 2000, replace=False)
        floor_pts = floor_pts[idx]
    centroid = floor_pts.mean(axis=0)
    _, _, Vt = np.linalg.svd(floor_pts - centroid, full_matrices=False)
    normal = Vt[-1]
    if normal[1] < 0:
        normal = -normal
    Y_up = np.array([0.0, 1.0, 0.0])
    axis = np.cross(normal, Y_up)
    sin_a = float(np.linalg.norm(axis))
    cos_a = float(np.dot(normal, Y_up))
    if sin_a < 1e-6:
        return pts
    axis /= sin_a
    K = np.array([[       0, -axis[2],  axis[1]],
                  [ axis[2],        0, -axis[0]],
                  [-axis[1],  axis[0],        0]])
    R = np.eye(3) + sin_a * K + (1 - cos_a) * (K @ K)
    pts_rot = (R @ (pts - centroid).T).T + centroid
    floor_y_mean = pts_rot[floor_mask_flat, 1].mean()
    pts_rot[:, 1] -= floor_y_mean
    angle_deg = np.degrees(np.arctan2(sin_a, cos_a))
    print(f'  floor-plane fit: normal={normal.round(3)}, correction={angle_deg:.1f} deg')
    return pts_rot


# ---------------------------------------------------------------------------
# Apply semantic color override
# ---------------------------------------------------------------------------

def apply_semantic_colors(cols, ys_flat, xs_flat, obstacle_mask, drive_mask):
    """
    Override mau RGB goc theo nhan semantic.
    Uu tien: obstacle > drive_area > goc anh.
    """
    cols = cols.copy()
    is_drive    = drive_mask[ys_flat, xs_flat] > 0
    is_obstacle = obstacle_mask[ys_flat, xs_flat] > 0

    cols[is_drive]    = COLOR_FREE
    cols[is_obstacle] = COLOR_OBSTACLE   # obstacle de len sau -> uu tien cao hon
    return cols


# ---------------------------------------------------------------------------
# PLY writer
# ---------------------------------------------------------------------------

def write_ply(path, pts, cols):
    cols = cols.astype(np.uint8)
    with open(path, 'wb') as f:
        header = (
            "ply\nformat ascii 1.0\n"
            f"element vertex {len(pts)}\n"
            "property float x\nproperty float y\nproperty float z\n"
            "property uchar red\nproperty uchar green\nproperty uchar blue\n"
            "end_header\n"
        )
        f.write(header.encode())
        buf = np.column_stack([pts, cols])
        np.savetxt(f, buf, fmt='%.4f %.4f %.4f %d %d %d')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args = parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    device = pick_device(args.device)

    # infer_scale nhân với input_size rồi làm tròn xuống bội của 14 (patch size ViT)
    infer_size = max(14, int(args.input_size * args.infer_scale) // 14 * 14)
    print(f'device={device}  encoder={args.encoder}  infer_size={infer_size}px')

    t_load = time.time()
    model = DepthAnythingV2(**CFGS[args.encoder])
    ckpt  = os.path.join(ROOT, 'model', f'depth_anything_v2_{args.encoder}.pth')
    model.load_state_dict(torch.load(ckpt, map_location='cpu', weights_only=True))
    model = model.to(device).eval()
    _tick('model load', t_load)

    if device == 'cuda':
        t = time.time()
        dummy = np.zeros((infer_size, infer_size, 3), dtype=np.uint8)
        model.infer_image(dummy, infer_size)
        torch.cuda.synchronize()
        _tick('cuda warmup', t)

    if os.path.isfile(args.img):
        files = [args.img]
    else:
        files = sorted(glob.glob(os.path.join(args.img, '*.jpg')) +
                       glob.glob(os.path.join(args.img, '*.png')))
    if not files:
        print(f'Khong tim thay anh trong: {args.img}')
        sys.exit(1)

    last_ply = None
    t_global = time.time()
    times_per_img = []

    for k, f in enumerate(files):
        img = cv2.imread(f)
        if img is None:
            print(f'[WARN] Khong doc duoc: {f}')
            continue
        H, W = img.shape[:2]
        t_img = time.time()
        print(f'[{k+1}/{len(files)}] {os.path.basename(f)}  ({W}x{H})')

        # ----- 1. Depth inference -----
        t = time.time()
        depth_raw = model.infer_image(img, infer_size)
        if device == 'cuda':
            torch.cuda.synchronize()
        depth_raw = cv2.resize(depth_raw.astype(np.float32), (W, H),
                               interpolation=cv2.INTER_LINEAR)
        _tick('1. depth inference', t)

        # ----- 2. Obstacle mask -----
        t = time.time()
        depth_obs = preprocess_depth(depth_raw, args.blur)
        floor_profile = build_floor_profile(depth_obs, args.floor_rows)
        arm_mask = None
        if not args.no_arm_mask:
            try:
                ax1, ay1, ax2, ay2 = [float(v) for v in args.arm_region.split(',')]
                arm_mask = (ax1, ay1, ax2, ay2)
            except ValueError:
                pass
        obstacle_mask = compute_obstacle_mask(
            depth_obs, floor_profile,
            gap=args.gap, gap_scale=args.gap_scale,
            morph_k=args.morph, min_area=args.min_area, ceil_cut=args.ceil_cut,
            arm_mask=arm_mask, arm_near=args.arm_near,
        )
        n_obs = int((obstacle_mask > 0).sum())
        _tick(f'2. obstacle mask ({n_obs}px)', t)

        # ----- 3. Drive area mask -----
        t = time.time()
        depth_drive = preprocess_depth(depth_raw, args.drive_blur)
        drive_mask = compute_drive_mask(
            depth_drive,
            n_rays=args.n_rays, ray_step=args.ray_step,
            min_lin=args.min_lin, r2_min=args.r2_min,
        )
        n_free = int((drive_mask > 0).sum())
        _tick(f'3. drive area ({n_free}px)', t)

        # ----- 4. 3D projection -----
        t = time.time()
        pts, cols, ys_flat, xs_flat = depth_to_points(
            depth_raw, img, args.fov, args.stride, args.camera_height, args.tilt
        )
        _tick('4. 3D projection', t)

        # ----- 5. Floor flatten -----
        t = time.time()
        if not args.no_flatten:
            H_s = depth_raw.shape[0] // args.stride
            W_s = depth_raw.shape[1] // args.stride
            n_floor_rows = max(1, int(H_s * args.floor_rows))
            row_idx = np.arange(H_s * W_s) // W_s
            floor_mask_flat = row_idx >= (H_s - n_floor_rows)
            pts = fit_and_flatten(pts, floor_mask_flat)
        _tick('5. floor flatten', t)

        # ----- 6. Semantic color override -----
        t = time.time()
        cols = apply_semantic_colors(cols, ys_flat, xs_flat, obstacle_mask, drive_mask)
        _tick('6. semantic colors', t)

        # ----- 7. Ghi PLY -----
        t = time.time()
        name = os.path.splitext(os.path.basename(f))[0] + '_annotated.ply'
        out  = os.path.join(args.outdir, name)
        write_ply(out, pts, cols)
        _tick(f'7. write PLY ({len(pts)} pts)', t)

        t_total = time.time() - t_img
        times_per_img.append(t_total)
        last_ply = out
        print(f'  -> {out}')
        print(f'  TOTAL image: {t_total*1000:.1f} ms')

    if times_per_img:
        avg = sum(times_per_img) / len(times_per_img)
        print(f'\n--- SUMMARY ---')
        print(f'  {len(times_per_img)} anh | avg {avg*1000:.1f} ms/anh | total {(time.time()-t_global)*1000:.1f} ms')
    print('\nXong. Mo file .ply bang MeshLab / CloudCompare.')
    print('  Do = obstacle | Xanh = drive area | Goc = khong xac dinh')

    if args.show and last_ply:
        try:
            import open3d as o3d
            pcd = o3d.io.read_point_cloud(last_ply)
            o3d.visualization.draw_geometries([pcd])
        except ImportError:
            print('Chua cai open3d -> bo qua --show.')


if __name__ == '__main__':
    main()
