"""
Pipeline DAY DU: anh 2D -> point cloud 3D (.ply) bang Depth Anything V2,
CO DO THOI GIAN TUNG STEP.

6 step / moi anh:
  1 read     - cv2.imread
  2 prep     - image2tensor (resize 518 + normalize + to device)
  3 infer    - model.forward + interpolate ve kich thuoc goc (inference thuan)
  4 project  - depth + FOV -> toa do XYZ + mau
  5 ply      - ghi file .ply
  (step 0 load model do 1 lan luc khoi tao)

Vi du:
  python depth_to_3d_timed.py --img ../assets/right_1.jpg
  python depth_to_3d_timed.py --img ../assets --device cuda
"""
import os, sys, glob, time, argparse
import cv2, numpy as np, torch
import torch.nn.functional as F

from depth_anything_v2.dpt import DepthAnythingV2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # depth-anything/

CFGS = {
    'vits': {'encoder': 'vits', 'features': 64,  'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
}


def parse_args():
    p = argparse.ArgumentParser(description='2D image -> 3D point cloud (timed)')
    p.add_argument('--img', default=os.path.join(ROOT, 'assets'),
                   help='file anh hoac thu muc')
    p.add_argument('--encoder', default='vits', choices=['vits', 'vitb'])
    p.add_argument('--input-size', type=int, default=518)
    p.add_argument('--outdir', default=os.path.join(ROOT, 'output', 'pointcloud'))
    p.add_argument('--device', default='auto', choices=['auto', 'cuda', 'cpu'])
    p.add_argument('--fov', type=float, default=60.0, help='goc nhin ngang camera (do)')
    p.add_argument('--stride', type=int, default=2, help='1=giu het, 2=lay 1/4 diem')
    p.add_argument('--warmup', action='store_true',
                   help='chay 1 lan warmup truoc khi do (CUDA on dinh hon)')
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


class Timer:
    """Dem gio co dong bo CUDA. Dung: with Timer(dev) as t: ...  -> t.ms"""
    def __init__(self, device):
        self.device = device
        self.ms = 0.0
    def __enter__(self):
        if self.device == 'cuda':
            torch.cuda.synchronize()
        self.t0 = time.perf_counter()
        return self
    def __exit__(self, *a):
        if self.device == 'cuda':
            torch.cuda.synchronize()
        self.ms = (time.perf_counter() - self.t0) * 1000.0
        return False


# ---------- STEP 2: preprocess ----------
def step_prep(model, img_bgr, input_size):
    """resize + normalize + to tensor + to device. Tra (tensor, (h,w) goc)."""
    return model.image2tensor(img_bgr, input_size)


# ---------- STEP 3: inference thuan ----------
@torch.no_grad()
def step_infer(model, tensor, orig_hw):
    h, w = orig_hw
    depth = model.forward(tensor)
    depth = F.interpolate(depth[:, None], (h, w), mode='bilinear', align_corners=True)[0, 0]
    return depth.cpu().numpy()


# ---------- STEP 4: back-projection ----------
def step_project(depth, color_bgr, fov_deg, stride):
    """Chieu nguoc pixel + depth ra toa do 3D (camera o goc toa do)."""
    H, W = depth.shape
    fx = fy = (W / 2.0) / np.tan(np.deg2rad(fov_deg) / 2.0)  # tieu cu uoc luong tu FOV
    cx, cy = W / 2.0, H / 2.0

    # Depth Anything tra ve "do gan" (lon = gan). Dao thanh khoang cach.
    d = depth.astype(np.float32)
    d = d.max() - d
    d = d / (d.max() + 1e-8) * 5.0 + 0.5   # ~0.5..5.5 (don vi tuong doi)

    ys, xs = np.mgrid[0:H:stride, 0:W:stride]
    z = d[ys, xs]
    X = (xs - cx) * z / fx
    Y = -(ys - cy) * z / fy
    Z = -z

    pts = np.stack([X, Y, Z], axis=-1).reshape(-1, 3)
    cols = color_bgr[ys, xs][:, :, ::-1].reshape(-1, 3)  # BGR->RGB
    return pts, cols


# ---------- STEP 5: write PLY ----------
def step_write_ply(path, pts, cols):
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
        np.savetxt(f, np.column_stack([pts, cols]), fmt='%.4f %.4f %.4f %d %d %d')


def fmt_row(name, t, npts):
    return (f"{name:<16} | read {t['read']:6.1f} | prep {t['prep']:6.1f} | "
            f"infer {t['infer']:7.1f} | project {t['project']:6.1f} | "
            f"ply {t['ply']:7.1f} | total {t['total']:7.1f} ms | {npts} pts")


def main():
    args = parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    device = pick_device(args.device)
    print(f'device={device} encoder={args.encoder} torch={torch.__version__}')

    # ===== STEP 0: load model =====
    with Timer(device) as t0:
        model = DepthAnythingV2(**CFGS[args.encoder])
        ckpt = os.path.join(ROOT, 'model', f'depth_anything_v2_{args.encoder}.pth')
        model.load_state_dict(torch.load(ckpt, map_location='cpu'))
        model = model.to(device).eval()
    print(f'[step0] load model: {t0.ms:.1f} ms')

    if os.path.isfile(args.img):
        files = [args.img]
    else:
        files = sorted(glob.glob(os.path.join(args.img, '*.jpg')) +
                       glob.glob(os.path.join(args.img, '*.png')))
    if not files:
        print('Khong tim thay anh.'); return

    # warmup (lan dau CUDA cap phat bo nho/kernel -> cham gia tao)
    if args.warmup:
        tmp = cv2.imread(files[0])
        tt, hw = step_prep(model, tmp, args.input_size)
        step_infer(model, tt, hw)
        print('[warmup] done')

    last_ply, agg = None, []
    for k, f in enumerate(files):
        t = {}
        with Timer(device) as r: img = cv2.imread(f)
        t['read'] = r.ms
        if img is None:
            print(f'  bo qua (loi doc): {f}'); continue

        with Timer(device) as r: tensor, orig_hw = step_prep(model, img, args.input_size)
        t['prep'] = r.ms

        with Timer(device) as r: depth = step_infer(model, tensor, orig_hw)
        t['infer'] = r.ms

        with Timer(device) as r: pts, cols = step_project(depth, img, args.fov, args.stride)
        t['project'] = r.ms

        name = os.path.splitext(os.path.basename(f))[0] + '.ply'
        out = os.path.join(args.outdir, name)
        with Timer(device) as r: step_write_ply(out, pts, cols)
        t['ply'] = r.ms

        t['total'] = t['read'] + t['prep'] + t['infer'] + t['project'] + t['ply']
        last_ply = out
        agg.append(t)
        print(fmt_row(os.path.basename(f), t, len(pts)))

    # ===== bang trung binh =====
    if agg:
        keys = ['read', 'prep', 'infer', 'project', 'ply', 'total']
        avg = {k: sum(d[k] for d in agg) / len(agg) for k in keys}
        print('-' * 70)
        print('TRUNG BINH/anh (ms): ' +
              '  '.join(f'{k}={avg[k]:.1f}' for k in keys) +
              f'   (n={len(agg)})')
    print(f'Xong -> {args.outdir}  (mo .ply bang MeshLab/CloudCompare/3D Viewer)')

    if args.show and last_ply:
        try:
            import open3d as o3d
            o3d.visualization.draw_geometries([o3d.io.read_point_cloud(last_ply)])
        except ImportError:
            print('Chua cai open3d -> bo qua --show. Cai: pip install open3d')


if __name__ == '__main__':
    main()
