#!/usr/bin/env python3
"""
stereo_calibrate.py  –  Browser-based stereo camera calibration

Usage:
  python3 tools/stereo_calibrate.py [options]

Options:
  --rows   INT   Inner corners dọc   (default 6)
  --cols   INT   Inner corners ngang (default 9)
  --square FLOAT Kích thước ô, mm    (default 25.0)
  --port   INT   HTTP port           (default 8081)
  --out    PATH  Output .yml file    (default calib/stereo.yml)

Mở browser: http://<jetson-ip>:8081

Quy trình:
  1. In checkerboard (9x6 inner corners = 10x7 ô vuông)
  2. Đặt checkerboard trước 2 camera – border xanh = cả 2 detect được
  3. Bấm [Capture] – thu thập 15-25 tư thế khác nhau
  4. Bấm [Calibrate] – tính toán, lưu file
  5. Dùng file calib: python3 map3d_server.py --calib calib/stereo.yml

Output (tương thích StereoCamera::loadCalibration):
  M1, D1  – left camera matrix + distortion
  M2, D2  – right camera matrix + distortion
  R, T    – stereo extrinsics
  R1, R2, P1, P2, Q – rectification maps
"""

import argparse
import json
import os
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np

# ─── Runtime config (set by argparse in main) ─────────────────────────────────
CFG = {
    "rows":   6,
    "cols":   9,
    "square": 25.0,   # mm
    "out":    "calib/stereo.yml",
}
LEFT_IDX  = 0
RIGHT_IDX = 2
WIDTH     = 1280   # capture at native res for better calibration accuracy
HEIGHT    = 960
DISP_W    = 640    # display / detection resolution
DISP_H    = 480
MIN_PAIRS = 15

# ─── Shared state ─────────────────────────────────────────────────────────────
_lock          = threading.Lock()
_stream_jpeg   = None
_corners_ready = False      # True = both cameras detect corners this frame
_pairs         = []         # list of (img_pts_l, img_pts_r)  – full res
_calib_status  = {"done": False, "rms": None, "msg": "–", "running": False}


# ─── Object-point template ────────────────────────────────────────────────────

def _make_objp(rows, cols, square_mm):
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_mm / 1000.0   # mm → metres
    return objp


# ─── Corner detection ─────────────────────────────────────────────────────────
_SUBPIX_CRIT = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

def _find_corners(gray, pattern):
    """Return refined corners or None."""
    ret, corners = cv2.findChessboardCorners(gray, pattern,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ret:
        return None
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), _SUBPIX_CRIT)
    return corners


def _sharpness(gray):
    """Laplacian variance – reject blurry frames."""
    return cv2.Laplacian(gray, cv2.CV_64F).var()


# ─── Capture thread ───────────────────────────────────────────────────────────

def capture_loop():
    global _stream_jpeg, _corners_ready

    rows  = CFG["rows"]
    cols  = CFG["cols"]
    pat   = (cols, rows)   # OpenCV: (width, height) = (cols, rows)
    scale = DISP_W / WIDTH

    cap_l = cv2.VideoCapture(LEFT_IDX)
    cap_r = cv2.VideoCapture(RIGHT_IDX)
    # Request native resolution for accuracy
    for cap in (cap_l, cap_r):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, 15)

    font     = cv2.FONT_HERSHEY_SIMPLEX
    prev_cap = 0   # debounce auto-capture

    while True:
        ok_l, frame_l = cap_l.read()
        ok_r, frame_r = cap_r.read()
        if not ok_l or not ok_r:
            time.sleep(0.05)
            continue

        # Resize to display resolution for detection and overlay
        disp_l = cv2.resize(frame_l, (DISP_W, DISP_H))
        disp_r = cv2.resize(frame_r, (DISP_W, DISP_H))
        gray_l = cv2.cvtColor(disp_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(disp_r, cv2.COLOR_BGR2GRAY)

        c_l = _find_corners(gray_l, pat)
        c_r = _find_corners(gray_r, pat)
        found = (c_l is not None) and (c_r is not None)

        # Draw corners
        cv2.drawChessboardCorners(disp_l, pat, c_l, c_l is not None)
        cv2.drawChessboardCorners(disp_r, pat, c_r, c_r is not None)

        # Border colour: green=both found, red=missing
        bcolor = (0, 220, 0) if found else (0, 0, 220)
        for disp in (disp_l, disp_r):
            cv2.rectangle(disp, (0, 0), (DISP_W-1, DISP_H-1), bcolor, 4)

        # Overlay text
        with _lock:
            n = len(_pairs)
            done = _calib_status["done"]
            rms  = _calib_status["rms"]

        status_txt = ("READY" if found else "searching...")
        cv2.putText(disp_l, f"L  {status_txt}", (10, 26), font, 0.65, bcolor, 2)
        cv2.putText(disp_r, f"R  {status_txt}", (10, 26), font, 0.65, bcolor, 2)
        cv2.putText(disp_l, f"pairs: {n}/{MIN_PAIRS}", (10, 54), font, 0.6, (200,200,50), 2)
        if done and rms is not None:
            cv2.putText(disp_l, f"RMS={rms:.3f}px", (10, DISP_H-12),
                        font, 0.6, (50, 255, 50), 2)

        # Pattern size hint
        cv2.putText(disp_r, f"board {cols}x{rows}  sq={CFG['square']}mm",
                    (10, 54), font, 0.55, (180, 180, 180), 1)

        combined = np.hstack([disp_l, disp_r])
        _, jpg = cv2.imencode(".jpg", combined, [cv2.IMWRITE_JPEG_QUALITY, 82])

        with _lock:
            _stream_jpeg   = jpg.tobytes()
            _corners_ready = found

        # Store corners at FULL resolution for actual calibration
        # (higher accuracy than display-res corners)
        if found:
            gray_full_l = cv2.cvtColor(frame_l, cv2.COLOR_BGR2GRAY)
            gray_full_r = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)
            c_full_l = _find_corners(gray_full_l, pat)
            c_full_r = _find_corners(gray_full_r, pat)
            with _lock:
                _corners_ready_full = (c_full_l, c_full_r)
        else:
            with _lock:
                _corners_ready_full = None

        # Store in module-level for POST /capture to pick up
        global _pending_full
        _pending_full = _corners_ready_full if found else None


_pending_full = None   # (corners_l_full, corners_r_full) or None


# ─── Calibration ─────────────────────────────────────────────────────────────

def run_calibration():
    global _calib_status
    with _lock:
        pairs  = list(_pairs)
        rows   = CFG["rows"]
        cols   = CFG["cols"]
        sq     = CFG["square"]
        outpath = CFG["out"]

    if len(pairs) < MIN_PAIRS:
        with _lock:
            _calib_status["msg"] = f"Cần ít nhất {MIN_PAIRS} pairs (hiện có {len(pairs)})"
        return

    with _lock:
        _calib_status = {"done": False, "rms": None,
                         "msg": "Đang tính toán...", "running": True}

    img_size  = (WIDTH, HEIGHT)
    objp      = _make_objp(rows, cols, sq)
    obj_pts   = [objp] * len(pairs)
    img_pts_l = [p[0] for p in pairs]
    img_pts_r = [p[1] for p in pairs]

    try:
        # ── Individual camera calibration ──────────────────────────────────────
        print(f"[calib] Calibrating left camera with {len(pairs)} views ...")
        rms_l, M1, D1, _, _ = cv2.calibrateCamera(
            obj_pts, img_pts_l, img_size, None, None)
        print(f"[calib] Left  RMS: {rms_l:.4f} px")

        print(f"[calib] Calibrating right camera ...")
        rms_r, M2, D2, _, _ = cv2.calibrateCamera(
            obj_pts, img_pts_r, img_size, None, None)
        print(f"[calib] Right RMS: {rms_r:.4f} px")

        # ── Stereo calibration ─────────────────────────────────────────────────
        print(f"[calib] Running stereoCalibrate ...")
        crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-6)
        rms, M1, D1, M2, D2, R, T, E, F = cv2.stereoCalibrate(
            obj_pts, img_pts_l, img_pts_r,
            M1, D1, M2, D2, img_size,
            criteria=crit,
            flags=cv2.CALIB_FIX_INTRINSIC,
        )
        print(f"[calib] Stereo RMS: {rms:.4f} px")

        # ── Stereo rectification ───────────────────────────────────────────────
        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
            M1, D1, M2, D2, img_size, R, T,
            alpha=0, newImageSize=img_size,
        )

        # Measured baseline from T vector (sanity check)
        baseline_m = float(np.linalg.norm(T))
        print(f"[calib] Measured baseline: {baseline_m*1000:.2f} mm")
        print(f"[calib] Left  fx={M1[0,0]:.1f}  fy={M1[1,1]:.1f}  "
              f"cx={M1[0,2]:.1f}  cy={M1[1,2]:.1f}")
        print(f"[calib] Right fx={M2[0,0]:.1f}  fy={M2[1,1]:.1f}")

        # ── Save ───────────────────────────────────────────────────────────────
        os.makedirs(os.path.dirname(os.path.abspath(outpath)), exist_ok=True)
        fs = cv2.FileStorage(outpath, cv2.FILE_STORAGE_WRITE)
        fs.write("rms",      rms)
        fs.write("baseline", baseline_m)
        fs.write("M1", M1);  fs.write("D1", D1)
        fs.write("M2", M2);  fs.write("D2", D2)
        fs.write("R",  R);   fs.write("T",  T)
        fs.write("R1", R1);  fs.write("R2", R2)
        fs.write("P1", P1);  fs.write("P2", P2)
        fs.write("Q",  Q)
        fs.write("image_width",  WIDTH)
        fs.write("image_height", HEIGHT)
        fs.release()
        print(f"[calib] Saved → {outpath}")

        with _lock:
            _calib_status = {
                "done": True, "rms": round(rms, 4),
                "baseline_mm": round(baseline_m * 1000, 2),
                "fx_l": round(float(M1[0, 0]), 1),
                "fx_r": round(float(M2[0, 0]), 1),
                "msg": f"OK  RMS={rms:.3f}px  baseline={baseline_m*1000:.1f}mm",
                "running": False,
                "file": outpath,
            }

    except Exception as e:
        print(f"[calib] ERROR: {e}")
        with _lock:
            _calib_status = {"done": False, "rms": None,
                             "msg": f"Error: {e}", "running": False}


# ─── HTML ─────────────────────────────────────────────────────────────────────

_HTML = r"""<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="utf-8"/>
<title>Stereo Calibration</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#0c0e14;color:#cde;font-family:monospace;padding:12px}
h2{color:#7ef;margin-bottom:8px}
.row{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin-bottom:8px}
img{max-width:100%;border:2px solid #333;display:block}
button{padding:6px 18px;border-radius:5px;border:1px solid #3a6;
       cursor:pointer;font-size:13px;transition:.2s}
#btnCap{background:#1a4a2a;color:#6f6}
#btnCap.ready{background:#1f6a3a;border-color:#4f9;color:#afa;
              box-shadow:0 0 8px #2f8}
#btnCap:disabled{background:#222;color:#555;border-color:#444;cursor:default}
#btnCalib{background:#2a2a4a;color:#88f;border-color:#44a}
#btnCalib:disabled{background:#222;color:#555;border-color:#444;cursor:default}
#btnClear{background:#3a1a1a;color:#f88;border-color:#a44}
.stat{background:#111;border:1px solid #2a3a4a;padding:8px 14px;
      border-radius:6px;min-width:160px}
.stat span{color:#7ef}
#msg{padding:8px 14px;border-radius:6px;background:#111;
     border:1px solid #2a4a2a;color:#afa;margin-top:8px;min-height:28px}
.step{background:#111;border:1px solid #2a3a5a;border-radius:6px;
      padding:10px 14px;margin-top:10px;font-size:12px;line-height:1.7}
.step b{color:#7ef}
</style>
</head>
<body>
<h2>Stereo Camera Calibration</h2>

<div class="row">
  <button id="btnCap"   onclick="capture()"  disabled>⊞ Capture</button>
  <button id="btnCalib" onclick="calibrate()" disabled>⚙ Calibrate</button>
  <button id="btnClear" onclick="clearAll()">✕ Clear</button>
  <div class="stat">Pairs: <span id="nPairs">0</span> / 15</div>
  <div class="stat">Status: <span id="detStatus">–</span></div>
  <div class="stat" id="rmsBox" style="display:none">
    RMS: <span id="rmsVal">–</span> px &nbsp; baseline: <span id="bsVal">–</span> mm
  </div>
</div>

<img id="stream" src="/stream" alt="camera stream"/>

<div id="msg">Chờ camera...</div>

<div class="step">
  <b>Hướng dẫn:</b><br>
  1. In checkerboard (vd: <a href="https://calib.io/pages/camera-calibration-pattern-generator" style="color:#7af" target="_blank">calib.io</a>) – kích thước ô đặt đúng với --square<br>
  2. Giữ checkerboard trước 2 camera → border <b style="color:#4f4">xanh</b> = cả 2 phát hiện được<br>
  3. Bấm <b>[Capture]</b> – thay đổi góc nghiêng, khoảng cách, vị trí cho mỗi lần (15-25 tư thế)<br>
  4. Sau ≥15 pairs → bấm <b>[Calibrate]</b> → file lưu tự động<br>
  5. Chạy map3d: <code>python3 tools/map3d_server.py --calib calib/stereo.yml</code>
</div>

<script>
let ready = false, nPairs = 0;

async function poll(){
  try{
    const r = await fetch('/status');
    const s = await r.json();
    ready  = s.ready;
    nPairs = s.pairs;

    document.getElementById('nPairs').textContent = nPairs;
    document.getElementById('detStatus').textContent =
      ready ? '✓ DETECTED' : 'searching...';
    document.getElementById('detStatus').style.color = ready ? '#4f4' : '#f84';

    const btnCap = document.getElementById('btnCap');
    btnCap.disabled = !ready;
    btnCap.className = ready ? 'ready' : '';

    document.getElementById('btnCalib').disabled =
      (nPairs < 15) || s.calib.running;

    if(s.calib.msg && s.calib.msg !== '–')
      document.getElementById('msg').textContent = s.calib.msg;

    if(s.calib.done){
      document.getElementById('rmsBox').style.display='';
      document.getElementById('rmsVal').textContent = s.calib.rms;
      document.getElementById('bsVal').textContent  = s.calib.baseline_mm;
    }
  }catch(e){}
}

async function capture(){
  const btn = document.getElementById('btnCap');
  btn.textContent = '...';
  try{
    const r = await fetch('/capture', {method:'POST'});
    const s = await r.json();
    if(s.ok){
      document.getElementById('msg').textContent =
        `Captured pair #${s.total}  (${s.msg})`;
    } else {
      document.getElementById('msg').textContent = '✗ ' + s.msg;
    }
  }catch(e){}
  btn.textContent = '⊞ Capture';
}

async function calibrate(){
  document.getElementById('btnCalib').disabled = true;
  document.getElementById('msg').textContent = '⚙ Calibrating... (có thể mất 10-30s)';
  await fetch('/calibrate', {method:'POST'});
}

async function clearAll(){
  if(!confirm('Xóa tất cả pairs đã chụp?')) return;
  await fetch('/clear', {method:'POST'});
  document.getElementById('msg').textContent = 'Đã xóa.';
  document.getElementById('rmsBox').style.display='none';
}

setInterval(poll, 400);
poll();
</script>
</body>
</html>"""


# ─── HTTP handler ─────────────────────────────────────────────────────────────

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def do_GET(self):
        if self.path == "/":
            body = _HTML.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with _lock:
                        jpg = _stream_jpeg
                    if jpg:
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                            + jpg + b"\r\n")
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass

        elif self.path == "/status":
            with _lock:
                body = json.dumps({
                    "ready": _corners_ready,
                    "pairs": len(_pairs),
                    "calib": _calib_status,
                }).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == "/capture":
            with _lock:
                pending = _pending_full
                ready   = _corners_ready

            if not ready or pending is None:
                body = json.dumps({"ok": False, "msg": "Corners not detected"}).encode()
            else:
                c_l, c_r = pending
                if c_l is None or c_r is None:
                    body = json.dumps({"ok": False,
                                       "msg": "Full-res detection failed"}).encode()
                else:
                    # Sharpness check on display-res (fast proxy)
                    with _lock:
                        _pairs.append((c_l, c_r))
                        total = len(_pairs)
                    body = json.dumps({
                        "ok": True, "total": total,
                        "msg": f"pair {total} saved",
                    }).encode()

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        elif self.path == "/calibrate":
            t = threading.Thread(target=run_calibration, daemon=True)
            t.start()
            self.send_response(200)
            self.end_headers()

        elif self.path == "/clear":
            with _lock:
                _pairs.clear()
                _calib_status.update({"done": False, "rms": None,
                                      "msg": "–", "running": False})
            self.send_response(200)
            self.end_headers()

        else:
            self.send_response(404)
            self.end_headers()


# ─── Main ─────────────────────────────────────────────────────────────────────

def get_local_ip():
    try:
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def main():
    parser = argparse.ArgumentParser(description="Stereo camera calibration server")
    parser.add_argument("--rows",   type=int,   default=6,
                        help="Inner corners dọc (default 6)")
    parser.add_argument("--cols",   type=int,   default=9,
                        help="Inner corners ngang (default 9)")
    parser.add_argument("--square", type=float, default=25.0,
                        help="Kích thước ô vuông, mm (default 25.0)")
    parser.add_argument("--port",   type=int,   default=8081)
    parser.add_argument("--out",    default="calib/stereo.yml",
                        help="Output calibration file")
    args = parser.parse_args()

    CFG["rows"]   = args.rows
    CFG["cols"]   = args.cols
    CFG["square"] = args.square
    CFG["out"]    = args.out

    print(f"[calib] Checkerboard: {args.cols}×{args.rows} inner corners, "
          f"square={args.square}mm")
    print(f"[calib] Capture resolution: {WIDTH}×{HEIGHT}")
    print(f"[calib] Output: {args.out}")
    print(f"[calib] Cần ít nhất {MIN_PAIRS} pairs")

    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()

    ip = get_local_ip()
    print(f"[calib] >>> Mở browser: http://{ip}:{args.port} <<<")
    print(f"[calib] Nhấn Ctrl+C để dừng.\n")

    server = HTTPServer(("0.0.0.0", args.port), Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[calib] Stopped.")


if __name__ == "__main__":
    main()
