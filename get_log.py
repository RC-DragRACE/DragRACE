#!/usr/bin/env python3
"""走行ログを Chromebook に渡すための窓口（車の Pi で実行する）

  cd ~/DragRACE
  python3 get_log.py              窓口を開く（ポート 8000）
  python3 get_log.py --port 8001  ポート番号を変える

窓口を開いたら、Chromebook の「DragRACE ログビューア」で車の名前を入れ、
次のどれかのボタンを押す。zip がダウンロードされる。

  最新の1走行          /zip?latest=1
  最新から N 走行      /zip?latest=N
  手元にないものすべて  /zip?missing=1&have=run_...,run_...
                        （have = ビューアに読み込んだことがある走行。それ以外を全部渡す）

動画はブラウザで再生できる形式(H.264)に、渡す直前に変換する（変換済みなら飛ばす）。
終わるときは Ctrl+C。
"""
import argparse
import html
import io
import os
import re
import shutil
import socket
import subprocess
import sys
import threading
import time
import zipfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs, unquote

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
RUN_RE = re.compile(r"^run_\d{8}_\d{6}$")
HOST = socket.gethostname()
FFMPEG = shutil.which("ffmpeg") is not None
henkan_lock = threading.Lock()


# ---------- 走行の一覧 ----------
def zen_run():
    """logs/ にある走行を新しい順に返す（csv がある走行だけ）"""
    if not os.path.isdir(LOG_DIR):
        return []
    runs = {f[:-4] for f in os.listdir(LOG_DIR) if f.endswith(".csv") and RUN_RE.match(f[:-4])}
    return sorted(runs, reverse=True)


def run_files(run):
    out = []
    for suffix in (".csv", "_settei.json", ".mp4"):
        p = os.path.join(LOG_DIR, run + suffix)
        if os.path.exists(p):
            out.append(p)
    return out


# ---------- 動画の変換 ----------
def douga_codec(path):
    try:
        r = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0",
                            "-show_entries", "stream=codec_name", "-of", "csv=p=0", path],
                           capture_output=True, text=True, timeout=20)
        return r.stdout.strip()
    except Exception:
        return ""


def douga_henkan(run):
    """run_日時.mp4 を H.264 にする。.avi しか無い場合も mp4 を作る"""
    if not FFMPEG:
        return "ffmpeg なし（動画は元のまま）"
    mp4 = os.path.join(LOG_DIR, run + ".mp4")
    avi = os.path.join(LOG_DIR, run + ".avi")
    src = mp4 if os.path.exists(mp4) else (avi if os.path.exists(avi) else None)
    if src is None:
        return "動画なし"
    if src == mp4 and douga_codec(mp4) == "h264":
        return "変換済み"
    tmp = os.path.join(LOG_DIR, run + ".tmp.mp4")
    r = subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", src, "-c:v", "libx264",
                        "-preset", "veryfast", "-crf", "23", "-pix_fmt", "yuv420p",
                        "-movflags", "+faststart", "-an", tmp], capture_output=True, text=True)
    if r.returncode != 0:
        if os.path.exists(tmp):
            os.remove(tmp)
        return "変換失敗（動画は元のまま）: " + r.stderr.strip()[:120]
    os.replace(tmp, mp4)
    return "H.264 に変換"


# ---------- 3種類の指定 ----------
def erabu(q):
    """クエリから渡す走行を決める。(走行リスト, zip名の説明) を返す"""
    runs = zen_run()
    if "missing" in q:
        have = set()
        for v in q.get("have", []):
            have |= {x for x in v.split(",") if RUN_RE.match(x)}
        return [r for r in runs if r not in have], "missing"
    if "latest" in q:
        try:
            n = max(1, min(999, int(q["latest"][0])))
        except ValueError:
            n = 1
        return runs[:n], f"latest{n}"
    if "run" in q:
        want = {x for v in q["run"] for x in v.split(",") if RUN_RE.match(x)}
        return [r for r in runs if r in want], "select"
    return [], ""


def zip_tsukuru(runs):
    with henkan_lock:
        for run in runs:
            print(f"  {run}: {douga_henkan(run)}", flush=True)
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_STORED) as z:
        for run in runs:
            for p in run_files(run):
                z.write(p, arcname=os.path.basename(p))
    return buf.getvalue()


# ---------- 画面 ----------
CSS = """body{font-family:sans-serif;margin:24px;color:#16202b;max-width:760px}h1{font-size:20px}
p{color:#5a6676;font-size:14px}a.b{display:inline-block;background:#1d5cc0;color:#fff;padding:7px 14px;
border-radius:6px;text-decoration:none;margin:0 8px 8px 0}table{border-collapse:collapse}
td{padding:6px 12px 6px 0;border-bottom:1px solid #dce2e9;font-size:14px}"""


def nichiji(run):
    d = run[4:]
    return f"{d[0:4]}/{d[4:6]}/{d[6:8]} {d[9:11]}:{d[11:13]}:{d[13:15]}"


def top_page():
    runs = zen_run()
    rows = "".join(f'<tr><td>{nichiji(r)}</td><td><a href="/zip?run={r}">この走行をダウンロード</a></td></tr>' for r in runs)
    return f"""<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>{html.escape(HOST)} の走行ログ</title><style>{CSS}</style>
<h1>{html.escape(HOST)} の走行ログ（{len(runs)} 走行）</h1>
<p>ふだんはログビューアのボタンから集めます。この画面からも直接ダウンロードできます。</p>
<p><a class="b" href="/zip?latest=1">最新の1走行</a><a class="b" href="/zip?latest=5">最新から5走行</a><a class="b" href="/zip?latest=999">すべて</a></p>
<table>{rows}</table>"""


def nai_page(msg):
    return f"""<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>{html.escape(HOST)}</title><style>{CSS}</style><h1>{html.escape(msg)}</h1>
<p>このタブは閉じてかまいません。</p>"""


class Handler(BaseHTTPRequestHandler):
    def send(self, body, ctype, name=None):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        if name:
            self.send_header("Content-Disposition", f'attachment; filename="{name}"')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        path = unquote(u.path)
        if path == "/":
            return self.send(top_page().encode("utf-8"), "text/html; charset=utf-8")
        if path == "/zip":
            runs, setsumei = erabu(parse_qs(u.query))
            if not runs:
                msg = "新しい走行はありません（すべて手元にあります）" if setsumei == "missing" else "走行ログが見つかりません"
                print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)
                return self.send(nai_page(msg).encode("utf-8"), "text/html; charset=utf-8")
            print(f"[{time.strftime('%H:%M:%S')}] {len(runs)} 走行を渡します（{setsumei}）", flush=True)
            body = zip_tsukuru(runs)
            # ファイル名: 車の名前__指定__渡した中で一番新しい走行.zip（ビューアが車の名前を読み取る）
            name = f"{HOST}__{setsumei}__{runs[0][4:]}.zip"
            return self.send(body, "application/zip", name)
        self.send_error(404)

    def log_message(self, *args):
        pass


def jibun_no_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))   # 実際には送信しない。使っている経路のIPを知るため
        return s.getsockname()[0]
    except Exception:
        return None
    finally:
        s.close()


def main():
    ap = argparse.ArgumentParser(description="走行ログを Chromebook に渡す窓口")
    ap.add_argument("--port", type=int, default=8000)
    a = ap.parse_args()

    runs = zen_run()
    print(f"ログの場所: {LOG_DIR}（{len(runs)} 走行）")
    if not FFMPEG:
        print("※ ffmpeg が入っていないので、動画はブラウザで再生できないままです。")
        print("  インターネットにつないで一度だけ:  sudo apt install -y ffmpeg")
    try:
        srv = ThreadingHTTPServer(("0.0.0.0", a.port), Handler)
    except OSError:
        print(f"ポート {a.port} は使われています。別の番号で:  python3 get_log.py --port 8001")
        sys.exit(1)

    ip = jibun_no_ip()
    print("\n窓口を開きました。ログビューアの「車の名前」に次を入れてください")
    print(f"  {HOST}" + ("" if a.port == 8000 else f":{a.port}  （.local は自動で付きます）"))
    if ip:
        print(f"  つながらないときは IP で:  {ip}" + ("" if a.port == 8000 else f":{a.port}"))
    print("\n終わったら Ctrl+C")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n終了しました")


if __name__ == "__main__":
    main()
