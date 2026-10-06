import os
import json
import subprocess
import http.server
import socketserver
from urllib.parse import urlparse, parse_qs

PORT = 8080
WEB_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(WEB_DIR, 'static')
BENCHMARK_BIN = os.path.join(WEB_DIR, '..', 'build', 'benchmark_cpp')
MESHES_DIR = os.path.abspath(os.path.join(WEB_DIR, '..', '..', 'src', 'yaskawa_workcell_description', 'meshes', 'visual'))

class RobotHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=STATIC_DIR, **kwargs)

    def do_HEAD(self):
        parsed_path = urlparse(self.path)
        if parsed_path.path.startswith('/meshes/'):
            filename = os.path.basename(parsed_path.path)
            mesh_path = os.path.join(MESHES_DIR, filename)
            if os.path.exists(mesh_path):
                self.send_response(200)
                self.send_header('Content-Type', 'model/stl')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Content-Length', str(os.path.getsize(mesh_path)))
                self.end_headers()
            else:
                self.send_error(404, f"Mesh file not found: {filename}")
        else:
            super().do_HEAD()

    def end_headers(self):
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()

    def do_GET(self):
        parsed_path = urlparse(self.path)
        if parsed_path.path == '/api/benchmark':
            self.run_benchmark_api()
        elif parsed_path.path.startswith('/meshes/'):
            filename = os.path.basename(parsed_path.path)
            mesh_path = os.path.join(MESHES_DIR, filename)
            if os.path.exists(mesh_path):
                self.send_response(200)
                self.send_header('Content-Type', 'model/stl')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Content-Length', str(os.path.getsize(mesh_path)))
                self.end_headers()
                with open(mesh_path, 'rb') as f:
                    self.wfile.write(f.read())
            else:
                self.send_error(404, f"Mesh file not found: {filename}")
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

class ReusableTCPServer(socketserver.TCPServer):
    def __init__(self, server_address, RequestHandlerClass, bind_and_activate=True):
        self.allow_reuse_address = True
        super().__init__(server_address, RequestHandlerClass, bind_and_activate)

def main():
    print(f"Starting Yaskawa GP8 3D Dashboard Web Server at http://localhost:{PORT}")
    print(f"Serving STL Mesh models from: {MESHES_DIR}")
    with ReusableTCPServer(("", PORT), RobotHandler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down server.")

if __name__ == '__main__':
    main()
