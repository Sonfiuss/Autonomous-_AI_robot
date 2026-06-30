"""
Phat hien vat can + vung di duoc bang DO CAO SO VOI MAT SAN (height-above-ground).

Thay cho ray-linearity (drive_area.py): back-project depth -> 3D, fit mat phang san,
roi phan loai moi pixel theo do cao so voi mat phang do.
  - obstacle  : h-obs  < height < h-max   (cao hon san -> vat can)
  - floor     : |height| <= h-floor       (sat mat san -> di duoc)
  - unknown   : con lai (vd qua cao = tran/qua dau robot, hoac depth khong tin)
Ngung la DO CAO VAT LY (cm) -> ha --h-obs de bat them vat thap, khong sinh nhieu
nhu R^2. Ket qua: mask vat can DAY DAC (khoet duoc vat o giua san) + luoi occupancy
bird's-eye (free / blocked / unknown) co flood-fill reachability tu vi tri robot.

Tai dung depth_to_3d.py: depth_to_points (back-project), fit_floor_plane (SVD), CFGS.

Vi du:
  python obstacle_grid.py --img ../assets/right_1.jpg --show
  python obstacle_grid.py --img ../assets/right_1.jpg --h-obs 0.05 --cell-size 0.1
  python obstacle_grid.py --img ../../deepmap/shots/shot_006.jpg --scale 1.0
"""
import os, argparse
import cv2, numpy as np, torch

from depth_anything_v2.dpt import DepthAnythingV2
from depth_to_3d import depth_to_points, fit_floor_plane, pick_device, CFGS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def parse_args():
    p = argparse.ArgumentParser(description='Height-above-ground obstacle + drive-area')
    p.add_argument('--img', default=os.path.join(ROOT, 'assets', 'right_1.jpg'))
    p.add_argument('--encoder', default='vits', choices=['vits', 'vitb'])
    p.add_argument('--input-size', type=int, default=518)
    p.add_argument('--device', default='auto', choices=['auto', 'cuda', 'cpu'])
    p.add_argument('--outdir', default=os.path.join(ROOT, 'output', 'obstacle_grid'))
    # --- projection (giong depth_to_3d) ---
    p.add_argument('--fov', type=float, default=60.0, help='goc nhin ngang camera (do)')
    p.add_argument('--stride', type=int, default=2, help='lay mau thua pixel (>=1)')
    p.add_argument('--camera-height', type=float, default=0.32,
                   help='chieu cao camera so voi san (don vi depth ~met). Default=0.32')
    p.add_argument('--tilt', type=float, default=15.0,
                   help='goc cui camera xuong (do). Default=15')
    p.add_argument('--scale', type=float, default=1.0,
                   help='nhan toan bo toa do 3D ve met that (tu deepmap/scale_calib). '
                        'Default=1.0 = don vi tuong doi.')
    # --- phan loai theo do cao ---
    p.add_argument('--bottom-frac', type=float, default=0.30,
                   help='ty le dong duoi anh dung lam mau san de fit plane. Default=0.30')
    p.add_argument('--h-floor', type=float, default=0.04,
                   help='|height| <= nguong nay = san (met). Default=0.04 (4cm)')
    p.add_argument('--h-obs', type=float, default=0.06,
                   help='height > nguong nay = vat can (met). HA XUONG de bat vat thap. '
                        'Default=0.06 (6cm)')
    p.add_argument('--h-max', type=float, default=2.0,
                   help='bo qua diem cao hon nguong nay (tran / tren dau robot). Default=2.0')
    # --- occupancy grid bird's-eye ---
    p.add_argument('--grid-range-x', type=float, default=2.0,
                   help='nua be ngang luoi (met) ve moi ben. Default=2.0')
    p.add_argument('--grid-range-z', type=float, default=4.0,
                   help='tam nhin truoc cua luoi (met). Default=4.0')
    p.add_argument('--cell-size', type=float, default=0.10,
                   help='canh o luoi (met). Default=0.10 (10cm)')
    p.add_argument('--min-pts-cell', type=int, default=2,
                   help='so diem toi thieu de mot o duoc tinh la co du lieu. Default=2')
    p.add_argument('--alpha', type=float, default=0.45, help='do trong suot overlay')
    p.add_argument('--show', action='store_true', help='mo cua so xem ket qua')
    return p.parse_args()


def load_model(encoder, device):
    model = DepthAnythingV2(**CFGS[encoder])
    ckpt = os.path.join(ROOT, 'model', f'depth_anything_v2_{encoder}.pth')
    model.load_state_dict(torch.load(ckpt, map_location='cpu', weights_only=True))
    return model.to(device).eval()


def signed_height(pts, normal, centroid):
    """Do cao co dau cua moi diem so voi mat phang (normal huong len)."""
    return (pts - centroid) @ normal


def fit_plane_robust(pts, floor_mask, h_floor, iters=2):
    """Fit plane tren dai day anh, roi tinh lai chi tren inlier (|h|<h_floor)
    de loai vat can lot vao vung mau san. Tra ve (normal, centroid, heights)."""
    normal, centroid = fit_floor_plane(pts, floor_mask)
    if normal is None:
        return None, None, None
    for _ in range(iters):
        h = signed_height(pts, normal, centroid)
        inlier = floor_mask & (np.abs(h) < h_floor)
        if inlier.sum() < 50:
            break
        n2, c2 = fit_floor_plane(pts, inlier)
        if n2 is None:
            break
        normal, centroid = n2, c2
    return normal, centroid, signed_height(pts, normal, centroid)


def ground_profile_residual(pts, heights, z_max, cell, low_pct=15.0, min_pts=20, smooth=3):
    """Do cao VUOT TREN SAN CUC BO theo tung dai cu ly Z.

    Depth-Anything cho depth khong metric -> san tai dung bi cong, height-so-voi-1-mat-
    phang tang dan theo khoang cach (san xa doc nham thanh vat can). Khac phuc: voi moi
    dai Z, lay percentile thap cua height lam cao do san g(Z); residual = height - g(Z).
    Cong/sai-scale he thong bi hap thu -> chi vat THUC SU noi len san moi duong tinh.
    """
    z = pts[:, 2]
    nb = max(1, int(np.ceil(z_max / cell)))
    binidx = np.clip((z / cell).astype(int), 0, nb - 1)
    ground = np.full(nb, np.nan)
    for b in range(nb):
        m = binidx == b
        if m.sum() >= min_pts:
            ground[b] = np.percentile(heights[m], low_pct)
    valid = ~np.isnan(ground)
    if valid.sum() >= 2:
        ground = np.interp(np.arange(nb), np.where(valid)[0], ground[valid])
    elif valid.any():
        ground[:] = ground[valid][0]
    else:
        ground[:] = 0.0
    if smooth > 1:
        ground = np.convolve(ground, np.ones(smooth) / smooth, mode='same')
    return heights - ground[binidx]


def classify(residual, h_floor, h_obs, h_max):
    """0=unknown, 1=floor, 2=obstacle. Phan loai theo do vuot tren san cuc bo."""
    cls = np.zeros(residual.shape, dtype=np.uint8)
    cls[np.abs(residual) <= h_floor] = 1
    cls[(residual > h_obs) & (residual <= h_max)] = 2
    return cls


def build_occupancy(pts, cls_flat, args):
    """Bird's-eye occupancy: bin (X, Z) cua diem floor/obstacle vao luoi.
    Tra ve grid (nz, nx) gia tri: 0=unknown, 1=free, 2=blocked, 3=free-reachable."""
    rx, rz, cs = args.grid_range_x, args.grid_range_z, args.cell_size
    nx = max(1, int(round(2 * rx / cs)))
    nz = max(1, int(round(rz / cs)))

    X, Z = pts[:, 0], pts[:, 2]
    col = np.floor((X + rx) / cs).astype(int)
    row = np.floor(Z / cs).astype(int)               # row 0 = sat robot
    valid = (col >= 0) & (col < nx) & (row >= 0) & (row < nz)

    obs_c = np.zeros((nz, nx), dtype=int)
    flr_c = np.zeros((nz, nx), dtype=int)
    o = valid & (cls_flat == 2)
    f = valid & (cls_flat == 1)
    np.add.at(obs_c, (row[o], col[o]), 1)
    np.add.at(flr_c, (row[f], col[f]), 1)

    m = args.min_pts_cell
    grid = np.zeros((nz, nx), dtype=np.uint8)          # 0 unknown
    grid[flr_c >= m] = 1                                # free
    grid[obs_c >= m] = 2                                # blocked (uu tien)

    # Flood-fill reachability, chi di qua o free. Sat robot la VUNG MU duoi camera
    # (tilt -> san gan nhat quan sat duoc cach robot mot doan), nen seed tu HANG GAN
    # NHAT thuc su co o free thay vi cu hang 0.
    free_rows = np.where((grid == 1).any(axis=1))[0]
    reach = np.zeros_like(grid, dtype=bool)
    if len(free_rows) == 0:
        stack = []
    else:
        r0 = int(free_rows[0])
        stack = [(r, c) for r in range(r0, min(r0 + 2, nz))
                 for c in range(nx) if grid[r, c] == 1]
    seen = set(stack)
    while stack:
        r, c = stack.pop()
        reach[r, c] = True
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < nz and 0 <= nc < nx and (nr, nc) not in seen \
                    and grid[nr, nc] == 1:
                seen.add((nr, nc))
                stack.append((nr, nc))
    grid[(grid == 1) & reach] = 3                      # free-reachable = drivable
    return grid


def render_grid(grid, cell_px=6):
    """Top-down: blocked=do, drivable=xanh la, free-khong-toi=olive, unknown=xam."""
    palette = {
        0: (70, 70, 70),     # unknown
        1: (60, 110, 60),    # free nhung khong toi duoc
        2: (40, 40, 210),    # blocked (BGR -> do)
        3: (40, 200, 60),    # drivable
    }
    nz, nx = grid.shape
    img = np.zeros((nz, nx, 3), dtype=np.uint8)
    for v, col in palette.items():
        img[grid == v] = col
    img = np.flipud(img)                                # xa o tren, robot o duoi
    img = cv2.resize(img, (nx * cell_px, nz * cell_px), interpolation=cv2.INTER_NEAREST)
    # Marker robot: giua canh duoi
    cv2.circle(img, (nx * cell_px // 2, nz * cell_px - cell_px), 5, (0, 255, 255), -1)
    cv2.putText(img, 'BEV', (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return img


def draw_overlay(img_bgr, cls_full, alpha):
    """To mau anh goc: san=xanh la, vat can=do, unknown=giu nguyen."""
    overlay = img_bgr.copy()
    color = np.zeros_like(img_bgr)
    color[cls_full == 1] = (60, 180, 60)    # floor xanh
    color[cls_full == 2] = (40, 40, 210)    # obstacle do
    m = cls_full > 0
    overlay[m] = (img_bgr[m] * (1 - alpha) + color[m] * alpha).astype(np.uint8)
    return overlay


def zone_banner(overlay, cls_full):
    """Bang L/C/R: CLEAR/BLOCKED theo ty le pixel vat can o nua duoi moi vung."""
    H, W = cls_full.shape
    zw = W // 3
    for i, label in enumerate(('L', 'C', 'R')):
        x0 = i * zw
        sub = cls_full[H // 2:, x0:x0 + zw]
        obs_ratio = (sub == 2).mean()
        status = 'BLOCKED' if obs_ratio > 0.06 else 'CLEAR'
        col = (0, 0, 210) if status == 'BLOCKED' else (0, 200, 0)
        cv2.rectangle(overlay, (x0 + 2, 4), (x0 + zw - 2, 30), col, -1)
        cv2.putText(overlay, f'{label}:{status}', (x0 + 6, 23),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


def main():
    args = parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    device = pick_device(args.device)
    print(f'device={device} encoder={args.encoder} scale={args.scale}')
    if args.scale == 1.0:
        print('  [luu y] --scale 1.0: nguong tinh theo DON VI TUONG DOI cua depth, '
              'khong phai met that. Dat --scale tu deepmap/scale_calib de chuan hoa cm.')

    model = load_model(args.encoder, device)

    img = cv2.imread(args.img)
    if img is None:
        raise FileNotFoundError(f'Khong doc duoc anh: {args.img}')
    H, W = img.shape[:2]

    print('Depth inference...')
    depth = model.infer_image(img, args.input_size)
    depth = cv2.resize(depth.astype(np.float32), (W, H), interpolation=cv2.INTER_LINEAR)

    # Back-project -> 3D (world frame: X phai, Y len, Z truoc; san ~ Y=0)
    pts, _ = depth_to_points(depth, img, args.fov, args.stride,
                             args.camera_height, args.tilt)
    pts *= args.scale

    # Luoi diem da strided -> kich thuoc de map nguoc ve pixel
    ys, xs = np.mgrid[0:H:args.stride, 0:W:args.stride]
    H_s, W_s = ys.shape

    # Mat na san: dai bottom-frac dong duoi cung
    n_floor = max(1, int(H_s * args.bottom_frac))
    row_idx = np.arange(H_s * W_s) // W_s
    floor_mask = row_idx >= (H_s - n_floor)

    normal, centroid, heights = fit_plane_robust(pts, floor_mask, args.h_floor)
    if normal is None:
        print('ERROR: khong du diem san de fit mat phang.')
        return
    print(f'  plane normal={normal.round(3)}  '
          f'inlier-san={int((np.abs(heights) < args.h_floor).sum())} diem')

    residual = ground_profile_residual(pts, heights, args.grid_range_z, args.cell_size)
    cls_flat = classify(residual, args.h_floor, args.h_obs, args.h_max)
    cls_small = cls_flat.reshape(H_s, W_s)
    cls_full = cv2.resize(cls_small, (W, H), interpolation=cv2.INTER_NEAREST)

    overlay = draw_overlay(img, cls_full, args.alpha)
    zone_banner(overlay, cls_full)

    grid = build_occupancy(pts, cls_flat, args)
    bev = render_grid(grid)
    n_drive = int((grid == 3).sum())
    n_block = int((grid == 2).sum())
    print(f'  occupancy: {n_drive} o di duoc, {n_block} o vat can')

    # Depth colormap de doi chieu
    dn = depth.max() - depth
    dn = (dn - dn.min()) / (dn.max() - dn.min() + 1e-8)
    depth_vis = cv2.applyColorMap((dn * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)

    bev_r = cv2.resize(bev, (int(bev.shape[1] * H / bev.shape[0]), H),
                       interpolation=cv2.INTER_NEAREST)
    combined = np.hstack([overlay, depth_vis, bev_r])

    stem = os.path.splitext(os.path.basename(args.img))[0]
    out_path = os.path.join(args.outdir, f'{stem}_obstacle.jpg')
    cv2.imwrite(out_path, combined)
    print(f'Saved: {out_path}')

    if args.show:
        if not os.environ.get('DISPLAY'):
            print('  [show] khong co DISPLAY (headless/SSH) -> bo qua cua so, '
                  f'xem anh da luu: {out_path}')
        else:
            try:
                scale = min(1.0, 1700 / combined.shape[1])
                cv2.imshow('Obstacle / Drive area  [q=thoat]',
                           cv2.resize(combined, (0, 0), fx=scale, fy=scale))
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            except cv2.error as e:
                print(f'  [show] khong mo duoc cua so ({e}); xem anh da luu: {out_path}')


if __name__ == '__main__':
    main()
