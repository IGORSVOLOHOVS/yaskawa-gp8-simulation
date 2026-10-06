import os
import json
import http.server
import socketserver

PORT = 8765
MANUAL_DIR = os.path.dirname(os.path.abspath(__file__))

class ManualTestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=MANUAL_DIR, **kwargs)

    def do_GET(self):
        if self.path == '/api/tests':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            with open(os.path.join(MANUAL_DIR, 'tests.json'), 'r') as f:
                self.wfile.write(f.read().encode('utf-8'))
        else:
            super().do_GET()

def main():
    print(f"Manual Test Server running on http://127.0.0.1:{PORT}")
    with socketserver.TCPServer(("127.0.0.1", PORT), ManualTestHandler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass

if __name__ == '__main__':
    main()
