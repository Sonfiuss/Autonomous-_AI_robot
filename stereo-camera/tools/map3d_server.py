#!/usr/bin/env python3
"""
map3d_server.py  –  Stereo 3D scanning + WebGL viewer over HTTP

Usage:
  Live scan : python3 tools/map3d_server.py [--serial /dev/ttyUSB0] [--port 8080]
  View .ply : python3 tools/map3d_server.py --ply scan.ply [--scale 0.001] [--port 8080]

Browser (máy local): http://<jetson-ip>:8080
  Kéo chuột trái  → xoay
  Scroll           → zoom
  Kéo chuột phải  → pan
  [Clear]          → xóa cloud
  [Save PLY]       → tải file điểm

Pipeline:
  Camera L/R → SGBM disparity → depth (baseline=54mm)
  + Servo pan/tilt (ESP32 UART) → spherical projection → world XYZ
  → point cloud → /cloud.bin → WebGL browser
"""

import argparse
import fcntl
import json
import os
import struct
import termios
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np

# ─── Config ──────────────────────────────────────────────────────────────────
LEFT_IDX   = 0
RIGHT_IDX  = 2
WIDTH      = 640
HEIGHT     = 480
BASELINE   = 0.054          # metres (54 mm)
FOV_H_HALF = 35.0           # degrees, camera half-FOV horizontal
FOV_V_HALF = 25.0           # degrees, camera half-FOV vertical
FOCAL_PX   = WIDTH / (2.0 * np.tan(np.radians(FOV_H_HALF)))
DEPTH_MIN  = 0.25           # metres – discard closer
DEPTH_MAX  = 8.0            # metres – discard farther
SKIP       = 4              # spatial downsample (every Nth pixel)
MAX_PTS    = 400_000        # hard cap on cloud size
NUM_DISP   = 64             # SGBM numDisparities
BLOCK_SZ   = 5              # SGBM blockSize (odd)
BAUD       = 115200

# ─── Shared state ────────────────────────────────────────────────────────────
_cld_lock = threading.Lock()
# (N, 4) float32: x, y, z, dist_from_origin
_cloud    = np.empty((0, 4), dtype=np.float32)

_srv_lock = threading.Lock()
_servo    = {"pan": 0.0, "tilt": 0.0, "ok": False}

_stats    = {"dfps": 0.0, "pts": 0, "pan": 0.0, "tilt": 0.0, "serial": False}
_stats_lk = threading.Lock()

# ─── SGBM matcher ────────────────────────────────────────────────────────────
_matcher = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=NUM_DISP,
    blockSize=BLOCK_SZ,
    P1=8  * 3 * BLOCK_SZ ** 2,
    P2=32 * 3 * BLOCK_SZ ** 2,
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=32,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
)

# ─── Precomputed pixel→angle lookup tables ────────────────────────────────────
_us = np.arange(0, WIDTH,  SKIP, dtype=np.float32)
_vs = np.arange(0, HEIGHT, SKIP, dtype=np.float32)
_UU, _VV = np.meshgrid(_us, _vs)
_ALPHA = (_UU / (WIDTH  - 1) - 0.5) * 2.0 * np.radians(FOV_H_HALF)
_BETA  = (_VV / (HEIGHT - 1) - 0.5) * 2.0 * np.radians(FOV_V_HALF)

# ─── Serial (termios, no pyserial) ───────────────────────────────────────────

def _open_serial(port: str):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs       = termios.tcgetattr(fd)
    attrs[0]    = termios.IGNBRK
    attrs[0]   &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
    attrs[1]    = 0
    attrs[2]    = (attrs[2] & ~termios.CSIZE) | termios.CS8
    attrs[2]   |= termios.CLOCAL | termios.CREAD
    attrs[2]   &= ~(termios.PARENB | termios.CSTOPB)
    try:
        attrs[2] &= ~termios.CRTSCTS
    except AttributeError:
        pass
    attrs[3]    = 0
    attrs[6][termios.VMIN]  = 0
    attrs[6][termios.VTIME] = 1
    speed = termios.B115200
    attrs[4] = speed
    attrs[5] = speed
    termios.tcflush(fd, termios.TCIFLUSH)
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags & ~os.O_NONBLOCK)
    return os.fdopen(fd, "rb+", buffering=0)


def serial_reader(port: str):
    """Reads 'P <pan> <tilt>' lines from ESP32 and updates _servo."""
    while True:
        try:
            ser = _open_serial(port)
            print(f"[serial] Connected to {port}")
            with _stats_lk:
                _stats["serial"] = True
            buf = b""
            while True:
                ch = ser.read(1)
                if not ch:
                    continue
                if ch == b"\n":
                    line = buf.decode("ascii", errors="ignore").strip()
                    buf = b""
                    if line.startswith("P "):
                        parts = line[2:].split()
                        if len(parts) >= 2:
                            pan, tilt = float(parts[0]), float(parts[1])
                            with _srv_lock:
                                _servo["pan"]  = pan
                                _servo["tilt"] = tilt
                                _servo["ok"]   = True
                            with _stats_lk:
                                _stats["pan"]  = pan
                                _stats["tilt"] = tilt
                else:
                    buf += ch
        except Exception as e:
            print(f"[serial] Error: {e}  – retrying in 3s")
            with _stats_lk:
                _stats["serial"] = False
            time.sleep(3)


# ─── Depth → 3D projection (mirrors MapBuilder.cpp logic) ────────────────────

def _project_depth(depth: np.ndarray, pan_deg: float, tilt_deg: float) -> np.ndarray:
    """Return (N, 4) float32 array of [x, y, z, dist] world-space points."""
    pan_r  = np.radians(pan_deg)
    tilt_r = np.radians(tilt_deg)

    r = depth[_VV.astype(int), _UU.astype(int)]   # shape (Hd, Wd)
    valid = np.isfinite(r) & (r > 0)

    r_v     = r[valid].astype(np.float32)
    alpha_v = _ALPHA[valid]
    beta_v  = _BETA[valid]

    theta = pan_r  + alpha_v   # azimuth
    phi   = tilt_r + beta_v    # elevation

    cos_phi = np.cos(phi)
    X = r_v * cos_phi * np.sin(theta)
    Y = r_v * np.sin(phi)
    Z = r_v * cos_phi * np.cos(theta)
    D = np.sqrt(X*X + Y*Y + Z*Z, dtype=np.float32)

    return np.stack([X, Y, Z, D], axis=-1)


def _add_to_cloud(new_pts: np.ndarray):
    global _cloud
    if len(new_pts) == 0:
        return
    with _cld_lock:
        merged = np.vstack([_cloud, new_pts]) if len(_cloud) else new_pts
        if len(merged) > MAX_PTS:
            merged = merged[-MAX_PTS:]
        _cloud = merged
    with _stats_lk:
        _stats["pts"] = len(merged)


# ─── Static PLY loader (display a saved scan.ply through this viewer) ──────────

def _load_ply(path: str, scale: float = None) -> np.ndarray:
    """Load an ASCII PLY (x y z) into an (N, 4) [x, y, z, dist] cloud for the viewer.

    The C++ stereo_scan writes points in MILLIMETRES in the body frame
    (X=forward, Y=right, Z=up). This viewer expects METRES and Y-up, so we:
      • auto-scale mm→m when coords look large (override with --scale),
      • remap body Z-up → viewer Y-up: (x,y,z)_view = (Y_body, Z_body, X_body).
    """
    with open(path, "r") as f:
        line = f.readline()
        if not line.startswith("ply"):
            raise ValueError(f"{path} is not a PLY file")
        n = 0
        fmt = "ascii"
        while True:
            line = f.readline()
            if not line:
                raise ValueError("Unexpected EOF in PLY header")
            t = line.split()
            if t and t[0] == "format":
                fmt = t[1]
            elif t and t[0] == "element" and t[1] == "vertex":
                n = int(t[2])
            elif t and t[0] == "end_header":
                break
        if fmt != "ascii":
            raise ValueError(f"Only ASCII PLY supported (got '{fmt}')")
        raw = np.loadtxt(f, dtype=np.float32, max_rows=n)

    if raw.ndim == 1:
        raw = raw.reshape(1, -1)
    xyz = raw[:, :3]

    if scale is None:
        p95 = float(np.percentile(np.abs(xyz), 95)) if len(xyz) else 0.0
        scale = 0.001 if p95 > 100.0 else 1.0
        print(f"[map3d] PLY auto-scale: p95(|coord|)={p95:.1f} → scale={scale}")
    xyz = xyz * scale

    # body (X=fwd, Y=right, Z=up) → viewer (x=right, y=up, z=fwd)
    x = xyz[:, 1].copy()
    y = xyz[:, 2].copy()
    z = xyz[:, 0].copy()

    # Recenter on the cloud's centroid (median = robust to outliers) so it sits
    # at the origin the orbit camera looks at — otherwise the whole cloud is
    # forward of the camera and nothing is visible.
    if len(x):
        x -= np.median(x)
        y -= np.median(y)
        z -= np.median(z)
        ext = float(np.percentile(np.sqrt(x * x + y * y + z * z), 95))
        print(f"[map3d] PLY recentred on centroid; 95%% extent ~{ext:.2f} m")

    d = np.sqrt(x * x + y * y + z * z, dtype=np.float32)
    return np.stack([x, y, z, d], axis=-1).astype(np.float32)


# ─── Capture loop ─────────────────────────────────────────────────────────────

def capture_loop():
    cap_l = cv2.VideoCapture(LEFT_IDX)
    cap_r = cv2.VideoCapture(RIGHT_IDX)
    for cap in (cap_l, cap_r):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap_l.isOpened() or not cap_r.isOpened():
        print("[capture] Cannot open cameras – check /dev/video0 and /dev/video2")
        return

    t0, count = time.time(), 0
    print("[capture] Camera loop started")

    while True:
        ok_l, frame_l = cap_l.read()
        ok_r, frame_r = cap_r.read()
        if not ok_l or not ok_r:
            time.sleep(0.05)
            continue

        # Resize to target resolution if camera delivers larger frames
        if frame_l.shape[1] != WIDTH or frame_l.shape[0] != HEIGHT:
            frame_l = cv2.resize(frame_l, (WIDTH, HEIGHT))
            frame_r = cv2.resize(frame_r, (WIDTH, HEIGHT))

        gray_l = cv2.cvtColor(frame_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)

        disp16 = _matcher.compute(gray_l, gray_r)
        disp   = disp16.astype(np.float32) / 16.0

        with np.errstate(divide="ignore", invalid="ignore"):
            depth = np.where(disp > 1.0, BASELINE * FOCAL_PX / disp, np.nan)
        depth[(depth < DEPTH_MIN) | (depth > DEPTH_MAX)] = np.nan

        with _srv_lock:
            pan  = _servo["pan"]
            tilt = _servo["tilt"]

        new_pts = _project_depth(depth, pan, tilt)
        _add_to_cloud(new_pts)

        count += 1
        dt = time.time() - t0
        if dt >= 1.0:
            with _stats_lk:
                _stats["dfps"] = round(count / dt, 1)
            count, t0 = 0, time.time()


# ─── Binary cloud serialiser ──────────────────────────────────────────────────

def _make_cloud_bin() -> bytes:
    """uint32 N  +  float32[N*4] (x,y,z,dist)."""
    with _cld_lock:
        data = _cloud.copy()
    N = len(data)
    return struct.pack("<I", N) + (data.tobytes() if N else b"")


def _make_ply() -> bytes:
    with _cld_lock:
        pts = _cloud[:, :3].copy()
    N = len(pts)
    header = (
        f"ply\nformat ascii 1.0\nelement vertex {N}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "end_header\n"
    ).encode()
    rows = "\n".join(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}" for p in pts) + "\n"
    return header + rows.encode()


# ─── HTML / WebGL template ────────────────────────────────────────────────────

_HTML = r"""<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="utf-8"/>
<title>Stereo 3D Map</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#0a0a0f;color:#cde;font-family:monospace;overflow:hidden}
#hud{position:fixed;top:0;left:0;right:0;display:flex;align-items:center;
     gap:12px;padding:8px 14px;background:rgba(0,0,0,.55);z-index:10;
     font-size:13px;flex-wrap:wrap}
#hud b{color:#7ef}
button{background:#1e3a4a;color:#adf;border:1px solid #3a6a8a;padding:4px 12px;
       border-radius:4px;cursor:pointer;font-size:12px}
button:hover{background:#2a5a7a}
canvas{display:block;width:100vw;height:100vh}
#hint{position:fixed;bottom:8px;left:0;right:0;text-align:center;
      font-size:11px;color:#556;pointer-events:none}
</style>
</head>
<body>
<div id="hud">
  <b>Stereo 3D Map</b>
  <span>pts: <b id="pts">0</b></span>
  <span>depth: <b id="dfps">–</b> fps</span>
  <span>pan: <b id="pan">0</b>° tilt: <b id="tlt">0</b>°</span>
  <span id="ser" style="color:#f66">serial: –</span>
  <button onclick="clearCloud()">Clear</button>
  <button onclick="savePLY()">Save PLY</button>
</div>
<canvas id="c"></canvas>
<div id="hint">Kéo trái: xoay &nbsp;|&nbsp; Kéo phải: pan &nbsp;|&nbsp; Scroll: zoom</div>

<script>
// ── WebGL setup ───────────────────────────────────────────────────────────────
const canvas = document.getElementById('c');
const gl = canvas.getContext('webgl');
if (!gl) { alert('WebGL not supported'); }

const VS = `
attribute vec4 a;
uniform mat4 M;
uniform float maxD;
varying float vt;
void main(){
  gl_Position = M * vec4(a.xyz, 1.0);
  float sz = 3.0 / max(0.1, gl_Position.w);
  gl_PointSize = clamp(sz, 1.0, 4.0);
  vt = clamp(a.w / maxD, 0.0, 1.0);
}`;

const FS = `
precision mediump float;
varying float vt;
void main(){
  float h = (1.0 - vt) * 0.667;
  float r = clamp(abs(h*6.0-3.0)-1.0, 0.0, 1.0);
  float g = clamp(2.0-abs(h*6.0-2.0), 0.0, 1.0);
  float b = clamp(2.0-abs(h*6.0-4.0), 0.0, 1.0);
  gl_FragColor = vec4(r,g,b, 1.0);
}`;

function mkShader(type, src){
  const s = gl.createShader(type);
  gl.shaderSource(s, src);
  gl.compileShader(s);
  if (!gl.getShaderParameter(s, gl.COMPILE_STATUS))
    throw gl.getShaderInfoLog(s);
  return s;
}
const prog = gl.createProgram();
gl.attachShader(prog, mkShader(gl.VERTEX_SHADER, VS));
gl.attachShader(prog, mkShader(gl.FRAGMENT_SHADER, FS));
gl.linkProgram(prog);
const loc_a    = gl.getAttribLocation(prog, 'a');
const loc_M    = gl.getUniformLocation(prog, 'M');
const loc_maxD = gl.getUniformLocation(prog, 'maxD');
const ptBuf    = gl.createBuffer();
gl.enable(gl.DEPTH_TEST);
gl.clearColor(0.04, 0.04, 0.08, 1);

// ── Matrix math ───────────────────────────────────────────────────────────────
function perspective(fovDeg, asp, near, far){
  const f = 1/Math.tan(fovDeg*Math.PI/360);
  const nf = 1/(near-far);
  return new Float32Array([
    f/asp,0,0,0,  0,f,0,0,
    0,0,(far+near)*nf,-1,
    0,0,2*far*near*nf,0]);
}
function lookAt([ex,ey,ez],[cx,cy,cz],[ux,uy,uz]){
  let zx=ex-cx,zy=ey-cy,zz=ez-cz;
  let zl=Math.hypot(zx,zy,zz); zx/=zl;zy/=zl;zz/=zl;
  let xx=uy*zz-uz*zy,xy=uz*zx-ux*zz,xz=ux*zy-uy*zx;
  let xl=Math.hypot(xx,xy,xz); xx/=xl;xy/=xl;xz/=xl;
  let yx=zy*xz-zz*xy,yy=zz*xx-zx*xz,yz=zx*xy-zy*xx;
  return new Float32Array([
    xx,yx,zx,0, xy,yy,zy,0, xz,yz,zz,0,
    -(xx*ex+xy*ey+xz*ez),-(yx*ex+yy*ey+yz*ez),-(zx*ex+zy*ey+zz*ez),1]);
}
function mul(a,b){
  const c=new Float32Array(16);
  for(let i=0;i<4;i++) for(let j=0;j<4;j++) for(let k=0;k<4;k++)
    c[i*4+j]+=a[i*4+k]*b[k*4+j];
  return c;
}

// ── Orbit camera ─────────────────────────────────────────────────────────────
let az=0.4, el=0.3, rad=5.0;
let panX=0, panY=0;       // view-space pan offset
let drag=null;

canvas.addEventListener('mousedown', e=>{
  drag={x:e.clientX,y:e.clientY,az,el,px:panX,py:panY,btn:e.button};
  e.preventDefault();
});
window.addEventListener('mousemove', e=>{
  if(!drag) return;
  const dx=(e.clientX-drag.x)/canvas.width;
  const dy=(e.clientY-drag.y)/canvas.height;
  if(drag.btn===0){              // left: orbit
    az=drag.az-dx*3.0;
    el=Math.max(-1.4,Math.min(1.4, drag.el+dy*2.0));
  } else {                       // right: pan
    panX=drag.px-dx*rad*2;
    panY=drag.py+dy*rad*2;
  }
});
window.addEventListener('mouseup',  ()=>{ drag=null; });
canvas.addEventListener('wheel', e=>{
  rad=Math.max(0.5, rad*Math.exp(e.deltaY*0.001));
  e.preventDefault();
},{passive:false});
canvas.addEventListener('contextmenu',e=>e.preventDefault());

// ── Render loop ───────────────────────────────────────────────────────────────
let nPts=0;
function resize(){
  canvas.width  = window.innerWidth;
  canvas.height = window.innerHeight;
  gl.viewport(0,0,canvas.width,canvas.height);
}
window.addEventListener('resize', resize);
resize();

function buildMVP(){
  const cx=rad*Math.cos(el)*Math.sin(az)+panX;
  const cy=rad*Math.sin(el)+panY;
  const cz=rad*Math.cos(el)*Math.cos(az);
  const V=lookAt([cx,cy,cz],[panX,panY,0],[0,1,0]);
  const P=perspective(60, canvas.width/canvas.height, 0.01, 200);
  return mul(P,V);
}

function render(){
  requestAnimationFrame(render);
  gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
  if(nPts===0) return;
  const MVP=buildMVP();
  gl.useProgram(prog);
  gl.bindBuffer(gl.ARRAY_BUFFER, ptBuf);
  gl.enableVertexAttribArray(loc_a);
  gl.vertexAttribPointer(loc_a,4,gl.FLOAT,false,16,0);
  gl.uniformMatrix4fv(loc_M,false,MVP);
  gl.uniform1f(loc_maxD, 6.0);
  gl.drawArrays(gl.POINTS,0,nPts);
}
render();

// ── Data fetch ────────────────────────────────────────────────────────────────
async function fetchCloud(){
  try{
    const r=await fetch('/cloud.bin');
    const ab=await r.arrayBuffer();
    if(ab.byteLength<4) return;
    const dv=new DataView(ab);
    const N=dv.getUint32(0,true);
    nPts=N;
    document.getElementById('pts').textContent=N.toLocaleString();
    if(N>0){
      const fa=new Float32Array(ab,4);
      gl.bindBuffer(gl.ARRAY_BUFFER,ptBuf);
      gl.bufferData(gl.ARRAY_BUFFER,fa,gl.DYNAMIC_DRAW);
    }
  }catch(e){}
}

async function fetchStatus(){
  try{
    const r=await fetch('/status');
    const s=await r.json();
    document.getElementById('dfps').textContent=s.dfps;
    document.getElementById('pan').textContent=s.pan.toFixed(1);
    document.getElementById('tlt').textContent=s.tilt.toFixed(1);
    const ser=document.getElementById('ser');
    ser.textContent='serial: '+(s.serial?'OK':'–');
    ser.style.color=s.serial?'#6f6':'#f66';
  }catch(e){}
}

function clearCloud(){
  fetch('/clear',{method:'POST'}).then(()=>{ nPts=0; });
}
function savePLY(){
  window.location='/save.ply';
}

setInterval(fetchCloud,  500);
setInterval(fetchStatus, 800);
fetchCloud();
fetchStatus();
</script>
</body>
</html>"""


# ─── HTTP handler ─────────────────────────────────────────────────────────────

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def do_GET(self):
        # Browsers cancel in-flight polls / refresh mid-download → broken pipe.
        # Harmless; swallow so it doesn't spam the console.
        try:
            self._route_get()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _route_get(self):
        if self.path == "/":
            body = _HTML.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        elif self.path == "/cloud.bin":
            body = _make_cloud_bin()
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        elif self.path == "/status":
            with _stats_lk:
                body = json.dumps(_stats).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        elif self.path == "/save.ply":
            body = _make_ply()
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Disposition",
                             'attachment; filename="scan.ply"')
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        try:
            self._route_post()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _route_post(self):
        if self.path == "/clear":
            global _cloud
            with _cld_lock:
                _cloud = np.empty((0, 4), dtype=np.float32)
            with _stats_lk:
                _stats["pts"] = 0
            self.send_response(200)
            self.end_headers()
            print("[server] Cloud cleared")
        else:
            self.send_response(404)
            self.end_headers()


# ─── Main ─────────────────────────────────────────────────────────────────────

def get_local_ip() -> str:
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
    parser = argparse.ArgumentParser(description="Stereo 3D map server")
    parser.add_argument("--serial", metavar="PORT",
                        help="ESP32 serial port (e.g. /dev/ttyUSB0)")
    parser.add_argument("--port", type=int, default=8080,
                        help="HTTP port (default 8080)")
    parser.add_argument("--baseline", type=float, default=BASELINE,
                        help="Stereo baseline in metres (default 0.054)")
    parser.add_argument("--ply", metavar="FILE",
                        help="Display a saved .ply (static mode — no camera/servo)")
    parser.add_argument("--scale", type=float, default=None,
                        help="Coord scale for --ply (default: auto mm→m)")
    args = parser.parse_args()

    if args.ply:
        # ── Static mode: just show a saved point cloud through the viewer ──
        global _cloud
        cloud = _load_ply(args.ply, args.scale)
        with _cld_lock:
            _cloud = cloud
        with _stats_lk:
            _stats["pts"] = len(cloud)
        print(f"[map3d] Loaded {len(cloud):,} points from {args.ply} (static viewer)")
    else:
        print(f"[map3d] Baseline    : {args.baseline*1000:.1f} mm")
        print(f"[map3d] Focal est.  : {FOCAL_PX:.1f} px  (FOV_H={FOV_H_HALF*2}°)")
        print(f"[map3d] Depth range : {DEPTH_MIN}–{DEPTH_MAX} m")
        print(f"[map3d] Max points  : {MAX_PTS:,}")

        if args.serial:
            t = threading.Thread(target=serial_reader, args=(args.serial,), daemon=True)
            t.start()
        else:
            print("[map3d] No --serial port → servo angles fixed at pan=0 tilt=0")
            print("[map3d]   Để dùng servo: thêm --serial /dev/ttyUSB0")

        cap_t = threading.Thread(target=capture_loop, daemon=True)
        cap_t.start()

    ip = get_local_ip()
    print(f"[map3d] HTTP server port {args.port}")
    print(f"[map3d] >>> Mở browser: http://{ip}:{args.port} <<<")
    print(f"[map3d] Nhấn Ctrl+C để dừng.\n")

    server = HTTPServer(("0.0.0.0", args.port), Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[map3d] Stopped.")


if __name__ == "__main__":
    main()
