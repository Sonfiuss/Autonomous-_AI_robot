import os, sys, glob, time, argparse, logging
from datetime import datetime
import cv2, numpy as np, matplotlib, torch

from depth_anything_v2.dpt import DepthAnythingV2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # depth-anything/

CFGS = {
    'vits': {'encoder': 'vits', 'features': 64,  'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
}


def parse_args():
    p = argparse.ArgumentParser(description='Depth Anything V2 - local runner with timing')
    p.add_argument('--encoder', default='vitb', choices=['vits', 'vitb', 'vitl'])
    p.add_argument('--input-size', type=int, default=518)
    p.add_argument('--assets', default=os.path.join(ROOT, 'assets'))
    p.add_argument('--outdir', default=os.path.join(ROOT, 'output'))
    # ========================= BẬT CUDA TẠI ĐÂY =========================
    # 'auto' = tự dùng GPU nếu có. Ép GPU: --device cuda . Ép CPU: --device cpu
    p.add_argument('--device', default='auto', choices=['auto', 'cuda', 'cpu'])
    # Tăng tốc trên GPU: --half  (FP16, nhanh hơn ~2x, cần CUDA)
    p.add_argument('--half', action='store_true', help='dùng FP16 (chỉ CUDA)')
    # ===================================================================
    p.add_argument('--grayscale', action='store_true')
    return p.parse_args()


def pick_device(choice):
    if choice == 'cuda':
        if not torch.cuda.is_available():
            raise RuntimeError('--device cuda nhưng torch.cuda.is_available() = False '
                               '(bản torch hiện tại là CPU-only, xem hướng dẫn cài).')
        return 'cuda'
    if choice == 'cpu':
        return 'cpu'
    return 'cuda' if torch.cuda.is_available() else 'cpu'


def main():
    args = parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    # ---- logging: ra cả console lẫn file ----
    log_path = os.path.join(args.outdir, 'run_timing.log')
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(message)s',
        handlers=[logging.FileHandler(log_path, encoding='utf-8'), logging.StreamHandler()],
    )
    log = logging.getLogger('da2')

    device = pick_device(args.device)
    use_half = args.half and device == 'cuda'
    log.info('==== RUN %s ====', datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
    log.info('encoder=%s input_size=%d device=%s half=%s torch=%s',
             args.encoder, args.input_size, device, use_half, torch.__version__)
    if device == 'cuda':
        log.info('GPU: %s', torch.cuda.get_device_name(0))

    t0 = time.time()
    model = DepthAnythingV2(**CFGS[args.encoder])
    ckpt = os.path.join(ROOT, 'model', f'depth_anything_v2_{args.encoder}.pth')
    model.load_state_dict(torch.load(ckpt, map_location='cpu'))
    model = model.to(device).eval()
    if use_half:
        model = model.half()
    log.info('model loaded in %.2fs (%s)', time.time() - t0, ckpt)

    cmap = matplotlib.colormaps.get_cmap('Spectral_r')
    files = sorted(glob.glob(os.path.join(args.assets, '*.jpg')) +
                   glob.glob(os.path.join(args.assets, '*.png')))
    if not files:
        log.warning('Khong tim thay anh trong %s', args.assets)
        return

    per_img = []
    for k, f in enumerate(files):
        img = cv2.imread(f)
        t = time.time()
        depth = model.infer_image(img, args.input_size)
        if device == 'cuda':
            torch.cuda.synchronize()  # cần để đo thời gian GPU chính xác
        dt = time.time() - t
        per_img.append(dt)

        d = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
        d = d.astype(np.uint8)
        if args.grayscale:
            vis = np.repeat(d[..., None], 3, axis=-1)
        else:
            vis = (cmap(d)[:, :, :3] * 255)[:, :, ::-1].astype(np.uint8)
        sep = np.ones((img.shape[0], 50, 3), dtype=np.uint8) * 255
        out = cv2.hconcat([img, sep, vis])
        name = os.path.splitext(os.path.basename(f))[0] + '_depth.png'
        cv2.imwrite(os.path.join(args.outdir, name), out)
        log.info('[%d/%d] %-16s infer=%.2fs -> %s', k + 1, len(files),
                 os.path.basename(f), dt, name)

    log.info('TONG %d anh | trung binh %.2fs/anh | tong infer %.2fs | output=%s',
             len(files), sum(per_img) / len(per_img), sum(per_img), args.outdir)


if __name__ == '__main__':
    main()
