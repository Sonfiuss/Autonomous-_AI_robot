"""
Gop nhieu anh (robot xoay tai cho) -> 1 point cloud 360 do quanh robot.

Y tuong: robot xoay tai cho bang step-motor. Moi frame gan voi 1 goc yaw biet truoc
(ghi trong manifest). Tung anh -> depth -> point cloud (khung camera), roi:
    P_world = Ry(yaw) . ( P_camera + [0,0,-cam_offset] )
Tat ca cloud quay ve chung 1 khung -> noi lai thanh 360 do.

LUU Y: Depth Anything la depth TUONG DOI, moi anh chuan hoa doc lap -> scale (met)
moi anh khac nhau => mep noi co the lech. Day la panorama 3D THO de hinh dung.
Dung --icp (can open3d) de tinh chinh, hoac chuyen sang metric-depth de chinh xac.

Manifest (csv, 1 dong / anh):
    # filename, angle_deg
    shot_000.jpg, 0
    shot_001.jpg, 30
    ...
Neu khong co manifest: dung --step-deg hoac chia deu 360 theo thu tu ten file.

Vi du:
  python merge_360.py --img ../assets --manifest ../assets/angles.csv --cam-offset 0.12
  python merge_360.py --img ../shots --step-deg 30 --icp --voxel 0.02
"""
import os, glob, time, argparse
import cv2, numpy as np, torch

# tai su dung pipeline depth da co (DRY)
from depth_to_3d_timed import (
    CFGS, pick_device, Timer, step_prep, step_infer, step_project,
)
from depth_anything_v2.dpt import DepthAnythingV2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def parse_args():
    p = argparse.ArgumentParser(description='Merge nhieu anh -> point cloud 360 do')
    p.add_argument('--img', default=os.path.join(ROOT, 'assets'),
                   help='thu muc chua cac anh chup khi xoay')
    p.add_argument('--manifest', default=None,
                   help='csv: "tenfile,goc_do" / dong. Neu thieu -> dung --step-deg')
    p.add_argument('--step-deg', type=float, default=None,
                   help='goc giua 2 anh ke (do). Mac dinh chia deu 360/N neu khong co manifest')
    p.add_argument('--cam-offset', type=float, default=0.0,
                   help='khoang cach camera ra truoc tam xoay (met)')
    p.add_argument('--ccw', action='store_true',
                   help='robot xoay nguoc chieu kim dong ho (mac dinh CW: dao dau goc)')
    # depth pipeline
    p.add_argument('--encoder', default='vits', choices=['vits', 'vitb'])
    p.add_argument('--input-size', type=int, default=518)
    p.add_argument('--device', default='auto', choices=['auto', 'cuda', 'cpu'])
    p.add_argument('--fov', type=float, default=60.0)
    p.add_argument('--stride', type=int, default=2)
    # merge / output
    p.add_argument('--out', default=os.path.join(ROOT, 'output', 'pointcloud', 'merged_360.ply'))
    p.add_argument('--voxel', type=float, default=0.0,
                   help='kich thuoc voxel downsample (met). 0 = tat. Can open3d')
    p.add_argument('--icp', action='store_true',
                   help='tinh chinh mep noi bang ICP (can open3d)')
    p.add_argument('--show', action='store_true')
    return p.parse_args()


def list_images(img_dir):
    if os.path.isfile(img_dir):
        return [img_dir]
    return sorted(glob.glob(os.path.join(img_dir, '*.jpg')) +
                  glob.glob(os.path.join(img_dir, '*.png')))


def load_angles(files, manifest, step_deg, ccw):
    """Tra dict {duong_dan_anh: goc_rad}. Uu tien manifest."""
    sign = 1.0 if ccw else -1.0
    by_name = {os.path.basename(f): f for f in files}
    angles = {}

    if manifest and os.path.isfile(manifest):
        used = []
        with open(manifest, 'r', encoding='utf-8') as fh:
            for line in fh:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = [x.strip() for x in line.replace('\t', ',').split(',')]
                if len(parts) < 2:
                    continue
                name, deg = parts[0], parts[1]
                if name.lower() in ('filename', 'file', 'name'):  # bo header
                    continue
                if name in by_name:
                    angles[by_name[name]] = sign * np.deg2rad(float(deg))
                    used.append(name)
        missing = [os.path.basename(f) for f in files if f not in angles]
        if missing:
            print(f'  [manifest] {len(used)} anh co goc, BO QUA {len(missing)} anh thieu goc: '
                  f'{missing[:5]}{"..." if len(missing) > 5 else ""}')
        return angles

    # khong co manifest -> deu nhau
    n = len(files)
    step = step_deg if step_deg is not None else (360.0 / n if n else 0.0)
    print(f'  [no manifest] chia deu, step={step:.2f} do, {n} anh')
    for i, f in enumerate(files):
        angles[f] = sign * np.deg2rad(step * i)
    return angles


def ry(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float32)


def transform_to_world(pts_cam, yaw, cam_offset):
    """P_world = Ry(yaw) . (P_cam + [0,0,-offset])."""
    p = pts_cam.copy()
    p[:, 2] -= cam_offset            # camera o truoc tam xoay -> diem xa hon theo -Z
    return p @ ry(yaw).T             # (N,3) @ (3,3)^T


def write_ply(path, pts, cols):
    cols = cols.astype(np.uint8)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'wb') as f:
        f.write((
            "ply\nformat ascii 1.0\n"
            f"element vertex {len(pts)}\n"
            "property float x\nproperty float y\nproperty float z\n"
            "property uchar red\nproperty uchar green\nproperty uchar blue\n"
            "end_header\n").encode())
        np.savetxt(f, np.column_stack([pts, cols]), fmt='%.4f %.4f %.4f %d %d %d')


# ---------- open3d (tuy chon) ----------
def try_open3d():
    try:
        import open3d as o3d
        return o3d
    except ImportError:
        return None


def to_o3d(o3d, pts, cols):
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
    pc.colors = o3d.utility.Vector3dVector((cols / 255.0).astype(np.float64))
    return pc


def icp_merge(o3d, clouds, voxel):
    """Ghep tuan tu tung cloud vao map bang ICP point-to-plane."""
    vox = voxel if voxel > 0 else 0.02
    radius = vox * 3
    def prep(pc):
        d = pc.voxel_down_sample(vox)
        d.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
        return d
    map_pc = clouds[0]
    map_d = prep(map_pc)
    for i in range(1, len(clouds)):
        src_d = prep(clouds[i])
        reg = o3d.pipelines.registration.registration_icp(
            src_d, map_d, vox * 1.5, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        clouds[i].transform(reg.transformation)
        map_pc += clouds[i]
        map_d = prep(map_pc)
        print(f'    icp anh {i}: fitness={reg.fitness:.3f} rmse={reg.inlier_rmse:.4f}')
    return map_pc


def main():
    args = parse_args()
    device = pick_device(args.device)
    print(f'device={device} encoder={args.encoder} torch={torch.__version__}')

    files = list_images(args.img)
    if not files:
        print('Khong tim thay anh.'); return
    angles = load_angles(files, args.manifest, args.step_deg, args.ccw)
    files = [f for f in files if f in angles]          # chi giu anh co goc
    if not files:
        print('Khong anh nao co goc -> dung.'); return

    # ===== load model =====
    with Timer(device) as t0:
        model = DepthAnythingV2(**CFGS[args.encoder])
        ckpt = os.path.join(ROOT, 'model', f'depth_anything_v2_{args.encoder}.pth')
        model.load_state_dict(torch.load(ckpt, map_location='cpu'))
        model = model.to(device).eval()
    print(f'[load model] {t0.ms:.1f} ms')

    o3d = try_open3d() if (args.icp or args.voxel > 0) else None
    if (args.icp or args.voxel > 0) and o3d is None:
        print('  [canh bao] chua cai open3d -> bo qua --icp/--voxel. Cai: pip install open3d')

    all_pts, all_cols, o3d_clouds = [], [], []
    timing = []
    for k, f in enumerate(files):
        t = {}
        with Timer(device) as r: img = cv2.imread(f)
        t['read'] = r.ms
        if img is None:
            print(f'  bo qua (loi doc): {f}'); continue
        with Timer(device) as r: tensor, hw = step_prep(model, img, args.input_size)
        t['prep'] = r.ms
        with Timer(device) as r: depth = step_infer(model, tensor, hw)
        t['infer'] = r.ms
        with Timer(device) as r: pts, cols = step_project(depth, img, args.fov, args.stride)
        t['project'] = r.ms
        with Timer(device) as r: wpts = transform_to_world(pts, angles[f], args.cam_offset)
        t['transform'] = r.ms

        if o3d is not None:
            o3d_clouds.append(to_o3d(o3d, wpts, cols))
        else:
            all_pts.append(wpts); all_cols.append(cols)
        timing.append(t)
        print(f'  [{k+1}/{len(files)}] {os.path.basename(f):<14} yaw={np.rad2deg(angles[f]):6.1f}d '
              f'| read {t["read"]:5.0f} prep {t["prep"]:5.0f} infer {t["infer"]:6.0f} '
              f'project {t["project"]:5.0f} transform {t["transform"]:5.1f} ms | {len(wpts)} pts')

    # ===== merge =====
    with Timer(device) as rm:
        if o3d is not None:
            if args.icp and len(o3d_clouds) > 1:
                print('  [merge] ICP...')
                merged = icp_merge(o3d, o3d_clouds, args.voxel)
            else:
                merged = o3d_clouds[0]
                for pc in o3d_clouds[1:]:
                    merged += pc
            if args.voxel > 0:
                merged = merged.voxel_down_sample(args.voxel)
            mpts = np.asarray(merged.points, dtype=np.float32)
            mcols = (np.asarray(merged.colors) * 255.0).astype(np.uint8)
        else:
            mpts = np.concatenate(all_pts, axis=0)
            mcols = np.concatenate(all_cols, axis=0)
    merge_ms = rm.ms

    with Timer(device) as rw:
        write_ply(args.out, mpts, mcols)
    write_ms = rw.ms

    # ===== bao cao =====
    if timing:
        keys = ['read', 'prep', 'infer', 'project', 'transform']
        avg = {k: sum(d[k] for d in timing) / len(timing) for k in keys}
        print('-' * 72)
        print('TRUNG BINH/anh (ms): ' + '  '.join(f'{k}={avg[k]:.1f}' for k in keys))
    print(f'[merge] {merge_ms:.1f} ms   [write] {write_ms:.1f} ms')
    print(f'TONG: {len(mpts)} diem 360 do -> {args.out}')

    if args.show:
        o3d = o3d or try_open3d()
        if o3d:
            o3d.visualization.draw_geometries([o3d.io.read_point_cloud(args.out)])
        else:
            print('Chua cai open3d -> bo qua --show.')


if __name__ == '__main__':
    main()
