import os
import json
import subprocess
import http.server
import socketserver
from urllib.parse import urlparse, parse_qs

def load_env():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    curr = script_dir
    env_path = None
    while curr and curr != os.path.dirname(curr):
        candidate = os.path.join(curr, '.claude', '.env')
        if os.path.exists(candidate):
            env_path = candidate
            break
        curr = os.path.dirname(curr)

    if env_path and os.path.exists(env_path):
        with open(env_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    if '#' in line and not ('"' in line or "'" in line):
                        line = line.split('#', 1)[0].strip()
                    if '=' in line:
                        k, v = line.split('=', 1)
                        k, v = k.strip(), v.strip().strip('\'"')
                        os.environ.setdefault(k, v)

    if 'GITHUB_KEY' in os.environ and 'GITHUB_TOKEN' not in os.environ:
        os.environ['GITHUB_TOKEN'] = os.environ['GITHUB_KEY']
    elif 'GITHUB_TOKEN' in os.environ and 'GITHUB_KEY' not in os.environ:
        os.environ['GITHUB_KEY'] = os.environ['GITHUB_TOKEN']

load_env()

PORT = 8080
WEB_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(WEB_DIR, 'static')
BENCHMARK_BIN = os.path.join(WEB_DIR, '..', 'build', 'benchmark_cpp')
MESHES_ROOT = os.path.abspath(os.path.join(WEB_DIR, '..', '..', 'src', 'yaskawa_workcell_description', 'meshes'))

def resolve_mesh_path(path_str):
    rel = path_str[len('/meshes/'):].lstrip('/')
    candidate = os.path.normpath(os.path.join(MESHES_ROOT, rel))
    if os.path.exists(candidate) and os.path.isfile(candidate) and candidate.startswith(MESHES_ROOT):
        return candidate
    # Fallback to visual directory
    visual_candidate = os.path.join(MESHES_ROOT, 'visual', os.path.basename(path_str))
    if os.path.exists(visual_candidate) and os.path.isfile(visual_candidate):
        return visual_candidate
    return None

class RobotHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=STATIC_DIR, **kwargs)

    def do_HEAD(self):
        parsed_path = urlparse(self.path)
        if parsed_path.path.startswith('/meshes/'):
            mesh_path = resolve_mesh_path(parsed_path.path)
            if mesh_path:
                self.send_response(200)
                self.send_header('Content-Type', 'model/stl')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Content-Length', str(os.path.getsize(mesh_path)))
                self.end_headers()
            else:
                self.send_error(404, f"Mesh file not found: {parsed_path.path}")
        else:
            super().do_HEAD()

    def end_headers(self):
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()

    def do_GET(self):
        parsed_path = urlparse(self.path)
        TEST_BIN = os.path.join(WEB_DIR, '..', 'build', 'test_cpp')
        if parsed_path.path == '/api/benchmark':
            self.run_benchmark_api()
        elif parsed_path.path == '/api/robot/verify':
            self.run_robot_verify_api(TEST_BIN)
        elif parsed_path.path.startswith('/meshes/'):
            mesh_path = resolve_mesh_path(parsed_path.path)
            if mesh_path:
                self.send_response(200)
                self.send_header('Content-Type', 'model/stl')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Content-Length', str(os.path.getsize(mesh_path)))
                self.end_headers()
                with open(mesh_path, 'rb') as f:
                    self.wfile.write(f.read())
            else:
                self.send_error(404, f"Mesh file not found: {parsed_path.path}")
        else:
            super().do_GET()

    def run_benchmark_api(self):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        if not os.path.exists(BENCHMARK_BIN):
            response = {"status": "error", "message": "Benchmark binary not found. Please build cpp_solver first."}
        else:
            try:
                res = subprocess.run([BENCHMARK_BIN], capture_output=True, text=True, timeout=15)
                response = {
                    "status": "success",
                    "output": res.stdout,
                    "error": res.stderr
                }
            except Exception as e:
                response = {"status": "error", "message": str(e)}

        self.wfile.write(json.dumps(response).encode('utf-8'))

    def run_robot_verify_api(self, test_bin):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        if not os.path.exists(test_bin):
            response = {"status": "error", "message": "Test binary not found. Run ./robot build release first."}
        else:
            try:
                res = subprocess.run([test_bin], capture_output=True, text=True, timeout=15)
                response = {
                    "status": "success",
                    "cpp_test_passed": ("ALL UNIT TESTS PASSED SUCCESSFULLY" in res.stdout),
                    "output": res.stdout
                }
            except Exception as e:
                response = {"status": "error", "message": str(e)}

        self.wfile.write(json.dumps(response).encode('utf-8'))

class ReusableTCPServer(socketserver.TCPServer):
    def __init__(self, server_address, RequestHandlerClass, bind_and_activate=True):
        self.allow_reuse_address = True
        super().__init__(server_address, RequestHandlerClass, bind_and_activate)

def main():
    print(f"Starting Yaskawa GP8 3D Dashboard Web Server at http://localhost:{PORT}")
    print(f"Serving STL Mesh models from: {MESHES_ROOT}")
    with ReusableTCPServer(("", PORT), RobotHandler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down server.")

if __name__ == '__main__':
    main()
