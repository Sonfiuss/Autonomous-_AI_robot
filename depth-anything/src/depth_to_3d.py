"""
Chuyen anh 2D -> point cloud 3D (.ply) bang Depth Anything V2.
Khong can open3d (xuat PLY thuan numpy). Neu co open3d + --show se mo cua so 3D.

Vi du:
  python depth_to_3d.py --img ../assets/right_1.jpg
  python depth_to_3d.py --img ../assets --device cuda --show
"""
import os, sys, glob, time, argparse
import cv2, numpy as np, torch

from depth_anything_v2.dpt import DepthAnythingV2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # depth-anything/

CFGS = {
    'vits': {'encoder': 'vits', 'features': 64,  'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
}


def parse_args():
    p = argparse.ArgumentParser(description='2D image -> 3D point cloud (Depth Anything V2)')
    p.add_argument('--img', default=os.path.join(ROOT, 'assets'),
                   help='file anh hoac thu muc')
    p.add_argument('--encoder', default='vits', choices=['vits', 'vitb'])
    p.add_argument('--input-size', type=int, default=518)
    p.add_argument('--outdir', default=os.path.join(ROOT, 'output', 'pointcloud'))
    # ===== BAT CUDA TAI DAY: --device cuda  (can torch ban CUDA) =====
    p.add_argument('--device', default='auto', choices=['auto', 'cuda', 'cpu'])
    # FOV camera (do) — anh huong "do mo" cua model 3D. Khong can chinh xac.
    p.add_argument('--fov', type=float, default=60.0, help='goc nhin ngang cua camera (do)')
    # Lay mau thua diem cho nhe (1 = giu het, 2 = lay 1/4 so diem)
    p.add_argument('--stride', type=int, default=2)
    p.add_argument('--camera-height', type=float, default=0.32,
                   help='chieu cao camera so voi san (met). Default=0.32 (32 cm).')
    p.add_argument('--tilt', type=float, default=15.0,
                   help='goc cui camera xuong so voi duong ngang (do). Default=15.')
    p.add_argument('--floor-rows', type=float, default=0.25,
                   help='ty le dong duoi anh dung lam mau san de fit plane (0..1). Default=0.25.')
    p.add_argument('--no-flatten', action='store_true',
                   help='tat floor-plane fitting, chi dung tilt cung.')
    p.add_argument('--show', action='store_true', help='mo cua so 3D (can open3d)')
    return p.parse_args()


def pick_device(choice):
    if choice == 'cuda':
        if not torch.cuda.is_available():
            raise RuntimeError('--device cuda nhung torch la ban CPU-only.')
        return 'cuda'
    if choice == 'cpu':
        return 'cpu'
    return 'cuda' if torch.cuda.is_available() else 'cpu'


def depth_to_points(depth, color_bgr, fov_deg, stride, camera_height=0.0, tilt_deg=0.0):
    """Chieu nguoc pixel + depth ra toa do 3D trong world frame.

    Quy trinh:
      1. Unproject pixel -> camera frame (Z forward theo truc camera)
      2. Rotate +tilt_deg quanh X: chuyen camera frame -> world frame
         (world Z = forward mat dat, world Y = len tren)
      3. Dich chuyen Y += camera_height: dat goc toa do tai mat san

    tilt_deg: goc camera cui xuong so voi duong ngang (15 do = nhin xuong 15 do).
    camera_height: don vi tuong doi (cung scale voi depth ~0.5..4.03).
    """
    H, W = depth.shape
    fx = fy = (W / 2.0) / np.tan(np.deg2rad(fov_deg) / 2.0)
    cx, cy = W / 2.0, H / 2.0

    # Depth Anything tra ve "do gan" (gia tri lon = gan). Dao lai thanh khoang cach.
    d = depth.astype(np.float32)
    d = d.max() - d
    d = d / (d.max() + 1e-8) * 5.0 + 0.5  # scale tho ~0.5..4.03

    ys, xs = np.mgrid[0:H:stride, 0:W:stride]
    z = d[ys, xs]

    # Buoc 1: camera frame — truc Z theo huong nhin cua camera
    Xc = (xs - cx) * z / fx
    Yc = -(ys - cy) * z / fy   # Y len tren trong camera frame
    Zc = z                      # Z di thang ve phia truoc camera

    # Buoc 2: xoay +tilt_deg quanh X de chuyen ve world frame
    # Camera cui xuong tilt_deg => world frame xoay nguoc +tilt_deg
    #   Xw =  Xc
    #   Yw =  Yc*cos(t) + Zc*sin(t)   (Y world = len tren theo mat dat)
    #   Zw = -Yc*sin(t) + Zc*cos(t)   (Z world = thang ve truoc theo mat dat)
    t = np.deg2rad(tilt_deg)
    Xw =  Xc
    Yw =  Yc * np.cos(t) + Zc * np.sin(t)
    Zw = -Yc * np.sin(t) + Zc * np.cos(t)

    pts = np.stack([Xw, Yw, Zw], axis=-1).reshape(-1, 3)

    # Buoc 3: dat goc toa do tai mat san
    pts[:, 1] += camera_height

    cols = color_bgr[ys, xs][:, :, ::-1].reshape(-1, 3)  # BGR->RGB
    return pts, cols


def fit_floor_plane(pts, floor_mask_flat, max_samples=2000):
    """SVD plane fit tren tap diem san. Sample truoc de tranh OOM."""
    floor_pts = pts[floor_mask_flat]
    if len(floor_pts) < 50:
        return None, None
    if len(floor_pts) > max_samples:
        idx = np.random.choice(len(floor_pts), max_samples, replace=False)
        floor_pts = floor_pts[idx]
    centroid = floor_pts.mean(axis=0)
    _, _, Vt = np.linalg.svd(floor_pts - centroid, full_matrices=False)
    normal = Vt[-1]          # eigenvector ung voi eigenvalue nho nhat = normal plane
    if normal[1] < 0:
        normal = -normal
    return normal, centroid


def flatten_to_floor(pts, normal, centroid):
    """Xoay point cloud sao cho floor plane tro thanh phang ngang (normal = Y+)."""
    Y_up = np.array([0.0, 1.0, 0.0])
    axis = np.cross(normal, Y_up)
    sin_a = np.linalg.norm(axis)
    cos_a = np.dot(normal, Y_up)
    if sin_a < 1e-6:
        return pts   # da nam ngang roi
    axis /= sin_a
    # Rodrigues rotation
    K = np.array([[    0, -axis[2],  axis[1]],
                  [ axis[2],     0, -axis[0]],
                  [-axis[1],  axis[0],     0]])
    R = np.eye(3) + sin_a * K + (1 - cos_a) * K @ K
    pts_centered = pts - centroid
    pts_rot = (R @ pts_centered.T).T + centroid
    # Dich chuyen Y sao cho diem san thap nhat = 0
    pts_rot[:, 1] -= pts_rot[floor_mask_flatten_global].mean() if False else \
                     (R @ (centroid - centroid).T + centroid)[1]
    return pts_rot


def flatten_floor_simple(pts, floor_pts_idx):
    """Fit plane + xoay + dich Y=0 cho diem san."""
    normal, centroid = fit_floor_plane(pts, floor_pts_idx)
    if normal is None:
        print('  [warn] Khong du diem san de fit plane, bo qua flatten.')
        return pts
    Y_up = np.array([0.0, 1.0, 0.0])
    axis = np.cross(normal, Y_up)
    sin_a = float(np.linalg.norm(axis))
    cos_a = float(np.dot(normal, Y_up))
    if sin_a < 1e-6:
        R = np.eye(3)
    else:
        axis /= sin_a
        K = np.array([[       0, -axis[2],  axis[1]],
                      [ axis[2],        0, -axis[0]],
                      [-axis[1],  axis[0],        0]])
        R = np.eye(3) + sin_a * K + (1 - cos_a) * (K @ K)
    pts_rot = (R @ (pts - centroid).T).T + centroid
    # Doi Y de mat san nam tai Y = 0
    floor_y_mean = pts_rot[floor_pts_idx, 1].mean()
    pts_rot[:, 1] -= floor_y_mean
    angle_deg = np.degrees(np.arctan2(sin_a, cos_a))
    print(f'  floor-plane fit: normal={normal.round(3)}, '
          f'correction={angle_deg:.1f} deg, floor_y_offset={floor_y_mean:.3f}')
    return pts_rot


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


def main():
    args = parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    device = pick_device(args.device)
    print(f'device={device} encoder={args.encoder} torch={torch.__version__}')

    model = DepthAnythingV2(**CFGS[args.encoder])
    ckpt = os.path.join(ROOT, 'model', f'depth_anything_v2_{args.encoder}.pth')
    model.load_state_dict(torch.load(ckpt, map_location='cpu'))
    model = model.to(device).eval()

    if os.path.isfile(args.img):
        files = [args.img]
    else:
        files = sorted(glob.glob(os.path.join(args.img, '*.jpg')) +
                       glob.glob(os.path.join(args.img, '*.png')))

    last_ply = None
    for k, f in enumerate(files):
        img = cv2.imread(f)
        t = time.time()
        depth = model.infer_image(img, args.input_size)
        if device == 'cuda':
            torch.cuda.synchronize()
        pts, cols = depth_to_points(depth, img, args.fov, args.stride,
                                    args.camera_height, args.tilt)

        if not args.no_flatten:
            # Xac dinh tap diem san: lay floor_rows% dong duoi cung cua anh
            H_s = depth.shape[0] // args.stride
            W_s = depth.shape[1] // args.stride
            n_floor_rows = max(1, int(H_s * args.floor_rows))
            # Mask phang (H_s*W_s,) — True = dong duoi cung
            row_idx = np.arange(H_s * W_s) // W_s   # dong tuong ung moi diem
            floor_mask = row_idx >= (H_s - n_floor_rows)
            pts = flatten_floor_simple(pts, floor_mask)

        name = os.path.splitext(os.path.basename(f))[0] + '.ply'
        out = os.path.join(args.outdir, name)
        write_ply(out, pts, cols)
        last_ply = out
        print(f'[{k+1}/{len(files)}] {os.path.basename(f)}: {len(pts)} diem, '
              f'{time.time()-t:.2f}s -> {out}')

    print('Xong. Mo file .ply bang MeshLab / CloudCompare / Windows 3D Viewer.')

    if args.show and last_ply:
        try:
            import open3d as o3d
            pcd = o3d.io.read_point_cloud(last_ply)
            o3d.visualization.draw_geometries([pcd])
        except ImportError:
            print('Chua cai open3d -> bo qua --show. Cai: pip install open3d')


if __name__ == '__main__':
    main()
