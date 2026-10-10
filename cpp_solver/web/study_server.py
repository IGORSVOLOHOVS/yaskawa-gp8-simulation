#!/usr/bin/env python3
"""Study console server: a dependency-free bridge between the browser and study_api.

The browser never talks to the C++ binary directly. This process keeps ONE
long-lived ``study_api`` child alive and relays newline-delimited JSON to it,
serialised behind a lock so a dragged slider cannot interleave two requests.
Standard library only: http.server, subprocess, threading, json.

    python cpp_solver/web/study_server.py [--port 8090]

Set STUDY_API to point at a binary outside the usual build directories.
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

WEB_DIR = os.path.dirname(os.path.abspath(__file__))
CPP_DIR = os.path.dirname(WEB_DIR)
REPO_DIR = os.path.dirname(CPP_DIR)
STATIC_DIR = os.path.join(WEB_DIR, "static", "study")
MESHES_DIR = os.path.join(
    REPO_DIR, "src", "yaskawa_workcell_description", "meshes", "visual"
)

BUILD_HINT = (
    "study_api binary not found. Build it with './robot build release' "
    "(or set STUDY_API to the binary path) and reload this page."
)

REQUEST_TIMEOUT_S = 20.0
MAX_BODY_BYTES = 4 * 1024 * 1024

MIME = {
    ".html": "text/html; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".svg": "image/svg+xml",
    ".png": "image/png",
    ".ico": "image/x-icon",
}


def candidate_binaries():
    """Every path the binary may live at, most specific first."""
    names = ("study_api", "study_api.exe")
    out = []
    override = os.environ.get("STUDY_API", "").strip()
    if override:
        out.append(os.path.abspath(override))
    roots = (
        os.path.join(CPP_DIR, "build"),
        os.path.join(REPO_DIR, "build"),
        os.path.join(CPP_DIR, "build", "Release"),
        os.path.join(CPP_DIR, "build", "release"),
    )
    for root in roots:
        for name in names:
            out.append(os.path.join(root, name))
    return out


def find_binary():
    for path in candidate_binaries():
        if os.path.isfile(path):
            return path
    return None


class StudyApi:
    """One long-lived child process, one request at a time, restarted if it dies."""

    def __init__(self):
        self._lock = threading.Lock()
        self._proc = None
        self._lines = queue.Queue()
        self._reader = None
        self._binary = None

    # -- lifecycle ---------------------------------------------------------
    def _spawn(self):
        binary = find_binary()
        if binary is None:
            raise FileNotFoundError(BUILD_HINT)
        self._binary = binary
        self._lines = queue.Queue()
        self._proc = subprocess.Popen(
            [binary],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            cwd=REPO_DIR,
            text=True,
            encoding="utf-8",
            bufsize=1,
        )
        proc = self._proc
        sink = self._lines

        def pump():
            try:
                for line in proc.stdout:
                    sink.put(line)
            except Exception:
                pass
            finally:
                sink.put(None)

        self._reader = threading.Thread(target=pump, daemon=True)
        self._reader.start()

    def _alive(self):
        return self._proc is not None and self._proc.poll() is None

    def _kill(self):
        proc, self._proc = self._proc, None
        if proc is None:
            return
        try:
            proc.kill()
        except Exception:
            pass

    def stop(self):
        with self._lock:
            self._kill()

    def binary_path(self):
        return self._binary or find_binary()

    # -- request path ------------------------------------------------------
    def request(self, payload):
        """Relay one request object and return the child's reply object."""
        echo_id = payload.get("id", 0)
        with self._lock:
            for attempt in (0, 1):
                if not self._alive():
                    self._kill()
                    try:
                        self._spawn()
                    except FileNotFoundError as exc:
                        return self._error(payload, str(exc))
                    except Exception as exc:
                        return self._error(payload, "cannot start study_api: %s" % exc)
                try:
                    line = json.dumps(payload, allow_nan=False) + "\n"
                except (TypeError, ValueError) as exc:
                    return self._error(payload, "request is not serialisable: %s" % exc)
                try:
                    self._proc.stdin.write(line)
                    self._proc.stdin.flush()
                except Exception:
                    self._kill()
                    if attempt == 0:
                        continue
                    return self._error(payload, "study_api stdin closed; restarted")
                reply = self._read_reply()
                if reply is None:
                    self._kill()
                    if attempt == 0:
                        continue
                    return self._error(
                        payload, "study_api did not answer in time; restarted"
                    )
                if isinstance(reply, dict):
                    reply.setdefault("id", echo_id)
                return reply
        return self._error(payload, "study_api unavailable")

    def _read_reply(self):
        """Return the first parseable JSON object the child writes, or None."""
        while True:
            try:
                line = self._lines.get(timeout=REQUEST_TIMEOUT_S)
            except queue.Empty:
                return None
            if line is None:  # child closed stdout
                return None
            line = line.strip()
            if not line:
                continue
            try:
                return json.loads(line)
            except ValueError:
                continue

    @staticmethod
    def _error(payload, message):
        return {
            "id": payload.get("id", 0),
            "ok": False,
            "module": payload.get("module", ""),
            "op": payload.get("op", ""),
            "error": message,
        }


API = StudyApi()


class StudyHandler(BaseHTTPRequestHandler):
    server_version = "GP8StudyConsole/1.0"
    protocol_version = "HTTP/1.1"

    # -- plumbing ----------------------------------------------------------
    def log_message(self, fmt, *args):
        sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))

    def _send(self, code, body, ctype):
        try:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            self.close_connection = True

    def _send_json(self, obj, code=200):
        try:
            body = json.dumps(obj).encode("utf-8")
        except (TypeError, ValueError):
            body = json.dumps({"ok": False, "error": "unserialisable reply"}).encode()
            code = 500
        self._send(code, body, "application/json; charset=utf-8")

    # -- routes ------------------------------------------------------------
    def do_GET(self):
        try:
            path = self.path.split("?", 1)[0].split("#", 1)[0]
            if path == "/api/describe":
                self._send_json(API.request({"id": 0, "op": "describe"}))
            elif path == "/api/status":
                binary = API.binary_path()
                self._send_json(
                    {
                        "ok": binary is not None,
                        "binary": binary,
                        "searched": candidate_binaries(),
                        "error": None if binary else BUILD_HINT,
                    }
                )
            elif path.startswith("/meshes/"):
                self._serve_mesh(path)
            else:
                self._serve_static(path)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            self.close_connection = True
        except Exception as exc:  # never take the server down with one request
            self._send_json({"ok": False, "error": "server error: %s" % exc}, 500)

    def do_HEAD(self):
        self.do_GET()

    def do_POST(self):
        try:
            path = self.path.split("?", 1)[0]
            if path != "/api/invoke":
                self._send_json({"ok": False, "error": "unknown route " + path}, 404)
                return
            try:
                length = int(self.headers.get("Content-Length") or 0)
            except ValueError:
                length = 0
            if length < 0 or length > MAX_BODY_BYTES:
                self._send_json({"ok": False, "error": "request body too large"}, 413)
                return
            raw = self.rfile.read(length) if length else b""
            try:
                payload = json.loads(raw.decode("utf-8") or "{}")
            except (ValueError, UnicodeDecodeError) as exc:
                self._send_json(
                    {"ok": False, "error": "malformed JSON body: %s" % exc}, 400
                )
                return
            if not isinstance(payload, dict):
                self._send_json(
                    {"ok": False, "error": "body must be a JSON object"}, 400
                )
                return
            self._send_json(API.request(payload))
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            self.close_connection = True
        except Exception as exc:
            self._send_json({"ok": False, "error": "server error: %s" % exc}, 500)

    # -- files -------------------------------------------------------------
    def _serve_mesh(self, path):
        name = os.path.basename(path)
        full = os.path.join(MESHES_DIR, name)
        if not name.lower().endswith(".stl") or not os.path.isfile(full):
            self._send_json({"ok": False, "error": "mesh not found: " + name}, 404)
            return
        with open(full, "rb") as handle:
            self._send(200, handle.read(), "model/stl")

    def _serve_static(self, path):
        rel = path.lstrip("/")
        if rel in ("", "study", "study/"):
            rel = "index.html"
        if rel.startswith("study/"):
            rel = rel[len("study/"):]
        full = os.path.normpath(os.path.join(STATIC_DIR, rel))
        if not full.startswith(STATIC_DIR) or not os.path.isfile(full):
            self._send(404, b"not found", "text/plain; charset=utf-8")
            return
        ctype = MIME.get(os.path.splitext(full)[1].lower(), "application/octet-stream")
        with open(full, "rb") as handle:
            self._send(200, handle.read(), ctype)


def main():
    parser = argparse.ArgumentParser(description="GP8 study console server")
    parser.add_argument(
        "--port", type=int, default=8090, help="HTTP port (default 8090)"
    )
    parser.add_argument("--host", default="", help="bind address (default all)")
    args = parser.parse_args()

    binary = find_binary()
    print("GP8 study console on http://localhost:%d/" % args.port)
    print("  meshes : %s" % MESHES_DIR)
    if binary:
        print("  binary : %s" % binary)
    else:
        print("  binary : MISSING -> %s" % BUILD_HINT)
    sys.stdout.flush()

    server = ThreadingHTTPServer((args.host, args.port), StudyHandler)
    server.daemon_threads = True
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down")
    finally:
        server.server_close()
        API.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
