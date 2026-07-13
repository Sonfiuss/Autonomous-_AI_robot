"""
Web viewer cho point cloud (.ply).
Chay server cuc bo + trang Three.js de xoay/zoom point cloud trong trinh duyet.

Vi du:
  python view_web.py                      # phuc vu thu muc output/pointcloud
  python view_web.py --dir ../output/pointcloud --port 8000

Sau do mo http://localhost:8000  (tu dong mo trinh duyet).
"""
import os, argparse, http.server, socketserver, webbrowser, threading, glob

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # depth-anything/

HTML = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Point Cloud Viewer</title>
<style>
  body{margin:0;font-family:sans-serif;background:#111;color:#eee;overflow:hidden}
  #ui{position:absolute;top:10px;left:10px;z-index:10;background:#000a;padding:10px;border-radius:8px}
  select,button{font-size:14px;padding:4px}
  #info{font-size:12px;color:#aaa;margin-top:6px}
</style></head>
<body>
<div id="ui">
  <label>File: <select id="files"></select></label>
  <button id="reload">Tai lai</button>
  <div id="info">Chuot trai: xoay | lan: zoom | chuot phai: di chuyen</div>
</div>
<script type="importmap">
{ "imports": {
  "three": "https://unpkg.com/three@0.160.0/build/three.module.js",
  "three/addons/": "https://unpkg.com/three@0.160.0/examples/jsm/"
}}
</script>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { PLYLoader } from 'three/addons/loaders/PLYLoader.js';

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x111111);
const camera = new THREE.PerspectiveCamera(60, innerWidth/innerHeight, 0.05, 500);
camera.position.set(0, 0, 6);
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setSize(innerWidth, innerHeight);
document.body.appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;        // quan tinh -> muot khi tha chuot
controls.dampingFactor = 0.08;        // cang nho cang "troi" lau
controls.zoomSpeed = 0.4;             // zoom tu tu, khong giat 1 phat
controls.rotateSpeed = 0.6;
controls.panSpeed = 0.8;
controls.zoomToCursor = true;         // zoom vao vi tri con tro
controls.minDistance = 0.5;           // khong lao vao ben trong point cloud
controls.maxDistance = 50;
scene.add(new THREE.AxesHelper(1));

let current = null;
const loader = new PLYLoader();
function load(name){
  if(current){ scene.remove(current); current.geometry.dispose(); current.material.dispose(); }
  loader.load('/ply/'+name, g => {
    g.computeBoundingBox();
    const c = g.boundingBox.getCenter(new THREE.Vector3());
    g.translate(-c.x, -c.y, -c.z);
    const m = new THREE.PointsMaterial({size:2.0, sizeAttenuation:false, vertexColors: g.hasAttribute('color')});
    if(!g.hasAttribute('color')) m.color.set(0x88ccff);
    current = new THREE.Points(g, m);
    scene.add(current);
  });
}

async function refresh(){
  const list = await (await fetch('/list')).json();
  const sel = document.getElementById('files');
  sel.innerHTML = '';
  list.forEach(f => { const o=document.createElement('option'); o.value=o.textContent=f; sel.appendChild(o); });
  if(list.length) load(list[0]);
}
document.getElementById('files').onchange = e => load(e.target.value);
document.getElementById('reload').onclick = refresh;
addEventListener('resize', () => {
  camera.aspect = innerWidth/innerHeight; camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});
(function animate(){ requestAnimationFrame(animate); controls.update(); renderer.render(scene,camera); })();
refresh();
</script>
</body></html>
"""


def make_handler(ply_dir):
    class H(http.server.BaseHTTPRequestHandler):
        def _send(self, body, ctype):
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/' or self.path == '/index.html':
                self._send(HTML.encode('utf-8'), 'text/html; charset=utf-8')
            elif self.path == '/list':
                import json
                files = sorted(os.path.basename(p) for p in glob.glob(os.path.join(ply_dir, '*.ply')))
                self._send(json.dumps(files).encode(), 'application/json')
            elif self.path.startswith('/ply/'):
                fn = os.path.basename(self.path[len('/ply/'):])
                fp = os.path.join(ply_dir, fn)
                if os.path.isfile(fp):
                    with open(fp, 'rb') as f:
                        self._send(f.read(), 'application/octet-stream')
                else:
                    self.send_error(404)
            else:
                self.send_error(404)

        def log_message(self, *a):
            pass
    return H


def main():
    ap = argparse.ArgumentParser(description='Web viewer cho point cloud (.ply)')
    ap.add_argument('--dir', default=os.path.join(ROOT, 'output', 'pointcloud/session_20260712'))
    ap.add_argument('--port', type=int, default=8000)
    ap.add_argument('--no-browser', action='store_true')
    args = ap.parse_args()

    if not os.path.isdir(args.dir):
        print('Khong thay thu muc:', args.dir); return
    files = glob.glob(os.path.join(args.dir, '*.ply'))
    print(f'Phuc vu {len(files)} file .ply tu {args.dir}')
    print(f'Mo trinh duyet: http://localhost:{args.port}  (Ctrl+C de dung)')

    handler = make_handler(args.dir)
    with socketserver.ThreadingTCPServer(('', args.port), handler) as httpd:
        if not args.no_browser:
            threading.Timer(0.8, lambda: webbrowser.open(f'http://localhost:{args.port}')).start()
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print('\nDa dung server.')


if __name__ == '__main__':
    main()
