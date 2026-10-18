#!/usr/bin/env python3
"""Companion server for examples/ftp.aur (Auric FTP on AuroraOS).

Auric apps only speak plain HTTP, so the 3DS cannot use the FTP protocol.
This script shares a folder over HTTP in the small JSON dialect ftp.aur
understands: browse folders, download files (Range requests are supported, as
the 3DS fetches big files in pieces). It is read-only.

    python tools/ftp_server.py [folder] [--port 8021] [--dest Aurora/FTP]
                               [--token SECRET] [--hidden]

Standard library only. Anyone on your network who can reach the port can read
the shared folder, so use --token (and the same TOKEN in ftp.aur) on a network
you do not trust.
"""
import argparse
import json
import os
import re
import socket
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

MAX_ITEMS = 400      # the 3DS parses at most ~16k JSON tokens per reply
NAME_COLS = 34       # what fits between the icon and the size column

ROOT = ""
DEST = "Aurora/FTP"
TOKEN = ""
HIDDEN = False
cwd = ""             # current folder, relative to ROOT, "/"-separated
lock = threading.Lock()


def resolve(rel):
    """Absolute path of `rel` inside ROOT, or None if it would leave it."""
    p = os.path.realpath(os.path.join(ROOT, *[x for x in rel.split("/") if x]))
    if p == ROOT or p.startswith(ROOT + os.sep):
        return p
    return None


def entries():
    """The current folder's rows: folders first, then files, by name."""
    base = resolve(cwd)
    if base is None:
        return []
    rows = []
    try:
        with os.scandir(base) as it:
            for e in it:
                if e.name.startswith(".") and not HIDDEN:
                    continue
                if e.is_symlink() and not (
                    os.path.realpath(e.path) == ROOT
                    or os.path.realpath(e.path).startswith(ROOT + os.sep)
                ):
                    continue    # a link that leaves the shared folder
                try:
                    is_dir = e.is_dir()
                    size = 0 if is_dir else e.stat().st_size
                except OSError:
                    continue
                rows.append((e.name, is_dir, size))
    except OSError:
        return []
    rows.sort(key=lambda r: (not r[1], r[0].lower()))
    return rows[:MAX_ITEMS]


def display(name):
    s = name.encode("ascii", "replace").decode("ascii")
    return s if len(s) <= NAME_COLS else s[: NAME_COLS - 1] + "~"


def sd_name(name):
    """A file name that is safe on the SD card's FAT file system."""
    s = re.sub(r'[<>:"/\\|?*\x00-\x1f]', "_", name.encode("ascii", "replace").decode("ascii"))
    s = s.strip(" .") or "file"
    return s[:100]


def listing():
    rows = entries()
    items = []
    for name, is_dir, size in rows:
        item = {"name": display(name), "dir": is_dir, "size": size}
        if not is_dir:
            item["dest"] = DEST.rstrip("/") + "/" + sd_name(name)
        items.append(item)
    return {"path": "/" + cwd, "count": len(items), "items": items}


class Handler(BaseHTTPRequestHandler):
    server_version = "AuricFTP/1"

    def log_message(self, fmt, *args):
        sys.stderr.write("%s  %s\n" % (self.address_string(), fmt % args))

    def reply(self, code, body=b"", ctype="text/plain", extra=None, send_body=True):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Connection", "close")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        if send_body:
            self.wfile.write(body)

    def do_HEAD(self):
        self.handle_request(False)

    def do_GET(self):
        self.handle_request(True)

    def handle_request(self, send_body):
        global cwd
        if TOKEN and self.headers.get("X-Token", "") != TOKEN:
            return self.reply(403, b"bad token\n", send_body=send_body)
        q = parse_qs(urlparse(self.path).query)
        cmd = q.get("cmd", ["ls"])[0]
        try:
            idx = int(q.get("i", ["-1"])[0])
        except ValueError:
            idx = -1
        with lock:
            rows = entries()
            if cmd == "cd" and 0 <= idx < len(rows) and rows[idx][1]:
                nxt = (cwd + "/" if cwd else "") + rows[idx][0]
                if resolve(nxt) is not None:
                    cwd = nxt
            elif cmd == "up":
                cwd = cwd.rpartition("/")[0]
            elif cmd == "dl":
                if 0 <= idx < len(rows) and not rows[idx][1]:
                    path = resolve((cwd + "/" if cwd else "") + rows[idx][0])
                    if path and os.path.isfile(path):
                        return self.send_file(path, send_body)
                return self.reply(404, b"no such file\n", send_body=send_body)
            body = json.dumps(listing()).encode()
        self.reply(200, body, "application/json", send_body=send_body)

    def send_file(self, path, send_body):
        size = os.path.getsize(path)
        start, end, code = 0, size - 1, 200
        m = re.match(r"bytes=(\d*)-(\d*)$", self.headers.get("Range", "").strip())
        if m and (m.group(1) or m.group(2)):
            if m.group(1):
                start = int(m.group(1))
                end = int(m.group(2)) if m.group(2) else size - 1
            else:                                   # "bytes=-N": the last N bytes
                start = max(0, size - int(m.group(2)))
            end = min(end, size - 1)
            if start > end or start >= size:
                return self.reply(416, b"", extra={"Content-Range": "bytes */%d" % size},
                                  send_body=send_body)
            code = 206
        length = end - start + 1 if size else 0
        self.send_response(code)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(length))
        self.send_header("Accept-Ranges", "bytes")
        if code == 206:
            self.send_header("Content-Range", "bytes %d-%d/%d" % (start, end, size))
        self.send_header("Connection", "close")
        self.end_headers()
        if not send_body or not length:
            return
        try:
            with open(path, "rb") as f:
                f.seek(start)
                left = length
                while left > 0:
                    chunk = f.read(min(65536, left))
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    left -= len(chunk)
        except (BrokenPipeError, ConnectionResetError):
            pass


def lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))
        return s.getsockname()[0]
    except OSError:
        return socket.gethostbyname(socket.gethostname())
    finally:
        s.close()


def main():
    global ROOT, DEST, TOKEN, HIDDEN
    ap = argparse.ArgumentParser(description="Share a folder with ftp.aur on the 3DS.")
    ap.add_argument("folder", nargs="?", default=".", help="folder to share (default: .)")
    ap.add_argument("--port", type=int, default=8021)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--dest", default=DEST, help="SD card folder downloads go to (must exist)")
    ap.add_argument("--token", default="", help="shared secret (also set TOKEN in ftp.aur)")
    ap.add_argument("--hidden", action="store_true", help="also list dotfiles")
    a = ap.parse_args()
    ROOT = os.path.realpath(os.path.expanduser(a.folder))
    if not os.path.isdir(ROOT):
        sys.exit("not a folder: " + ROOT)
    DEST, TOKEN, HIDDEN = a.dest, a.token, a.hidden
    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    print("Sharing %s (read-only)" % ROOT)
    print("Downloads go to SD:/%s on the 3DS: create that folder first." % DEST.strip("/"))
    print("Put this line in examples/ftp.aur, then rebuild:\n")
    print('  let SERVER: string = "http://%s:%d";' % (lan_ip(), a.port))
    if TOKEN:
        print('  let TOKEN: string = "%s";' % TOKEN)
    print("\nCtrl+C to stop.")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
