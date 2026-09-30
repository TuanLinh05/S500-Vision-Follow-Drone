"""Giao dien web kem token bao mat va luong video.
Fix A5: Them authentication token.
Fix B8: Stop latch uu tien tuyet doi.

v2.1:
  - luong MJPEG cung phai co token (truoc day ai vao duoc cong deu xem duoc)
  - moi endpoint doi trang thai deu la POST (chan <img src=...> CSRF)
  - kiem tra Origin/Referer la
  - bang trang thai an toan: fence, do cao, khoang cach, pin, thoi luong con lai
  - nut BO KHOA de huy muc tieu ma khong phai reload
"""

import hmac
import http.server
import json
import secrets
import threading
import time
import urllib.parse


class OperatorLink:
    def __init__(self):
        self.token = secrets.token_urlsafe(16)
        print(f"WEBUI TOKEN: {self.token}")
        self._t_alive = 0.0
        self._lk = threading.Lock()
        self.stop_latch = threading.Event()
        self.note = ""            # thong bao gan nhat cho nguoi van hanh

    def authed(self, params):
        t = params.get('t', [''])[0]
        return hmac.compare_digest(self.token, t)

    def beat(self):
        with self._lk:
            self._t_alive = time.monotonic()

    @property
    def age(self):
        with self._lk:
            if self._t_alive == 0.0:
                return 999.0
            return time.monotonic() - self._t_alive

    def request_stop(self):
        self.stop_latch.set()


HTML_TEMPLATE = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Drone Follow Control</title>
<style>
body { font-family: sans-serif; background: #1b1b1b; color: #eee; margin: 0; padding: 10px; }
.btn { padding: 16px 26px; font-size: 19px; font-weight: bold; margin: 6px;
       cursor: pointer; border: none; border-radius: 6px; touch-action: none; }
#engage { background: #b71c1c; color: #fff; }
#engage:active { background: #f44336; }
#disengage { background: #2e7d32; color: #fff; }
#unlock { background: #455a64; color: #fff; font-size: 15px; padding: 12px 18px; }
#banner { padding: 10px; border-radius: 6px; font-size: 18px; font-weight: bold;
          background: #333; margin-bottom: 8px; }
.ok { background: #1b5e20 !important; }
.warn { background: #ef6c00 !important; }
.bad { background: #b71c1c !important; }
#grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
        gap: 6px; margin-top: 8px; }
.cell { background: #262626; border: 1px solid #3a3a3a; border-radius: 5px; padding: 6px 8px; }
.cell b { display: block; font-size: 11px; color: #9e9e9e; font-weight: normal; }
.cell span { font-size: 17px; font-family: monospace; }
#video { max-width: 100%; height: auto; border: 2px solid #555; cursor: crosshair; }
#raw { font-family: monospace; white-space: pre-wrap; font-size: 11px;
       color: #888; margin-top: 8px; }
</style>
<script>
const TOKEN = '{TOKEN}';
let aliveInterval = null;

function tk(url) { return url + (url.includes('?') ? '&' : '?') + 't=' + TOKEN; }
function post(url) { return fetch(tk(url), {method: 'POST'}); }

function startEngage() {
    if (aliveInterval) return;
    fetch(tk('/alive'), {method: 'POST'});
    post('/engage');
    aliveInterval = setInterval(() => { fetch(tk('/alive'), {method: 'POST'}); }, 200);
}
function stopEngage() {
    if (aliveInterval) { clearInterval(aliveInterval); aliveInterval = null; }
    post('/disengage');
}
window.onblur = stopEngage;
document.addEventListener('visibilitychange', () => { if (document.hidden) stopEngage(); });

function pickTarget(e) {
    const rect = e.target.getBoundingClientRect();
    post('/pick?x=' + ((e.clientX - rect.left) / rect.width) +
         '&y=' + ((e.clientY - rect.top) / rect.height));
}

function cell(label, value, cls) {
    return '<div class="cell ' + (cls || '') + '"><b>' + label + '</b><span>' +
           value + '</span></div>';
}

setInterval(() => {
    fetch(tk('/stats')).then(r => r.json()).then(d => {
        const b = document.getElementById('banner');
        if (d.engaged) { b.className = 'bad'; b.innerText =
            'DANG DIEU KHIEN  yaw ' + d.yaw + ' do/s' +
            (d.engage_left !== null ? '   (con ' + d.engage_left + 's)' : ''); }
        else if (d.state !== 'idle') { b.className = 'warn';
            b.innerText = 'DANG VAO OFFBOARD: ' + d.state + '  ' + (d.note || ''); }
        else { b.className = ''; b.innerText = 'KHONG DIEU KHIEN  ' + (d.block || ''); }

        const fcls = d.fence === 'ok' ? 'ok' : (d.fence === 'warn' ? 'warn' : 'bad');
        let h = '';
        h += cell('HANG RAO', d.fence + (d.fence_why ? ' - ' + d.fence_why : ''), fcls);
        h += cell('CACH DIEM CHOT', d.dist_m + ' m');
        h += cell('DO CAO', d.alt_m + ' m');
        h += cell('TOC DO NGANG', d.speed_ms + ' m/s');
        h += cell('TROI', d.drift_m + ' m');
        h += cell('MODE PX4', d.mode + (d.mode_req ? ' -> ' + d.mode_req : ''),
                  d.failsafe ? 'bad' : '');
        h += cell('ARMED', d.armed ? 'YES' : 'no');
        h += cell('PIN', d.batt + ' V' + (d.batt_pct >= 0 ? ' / ' + d.batt_pct + '%' : ''));
        h += cell('RC ch' + d.rc_chan, d.rc_val);
        h += cell('MUC TIEU', d.target);
        h += cell('SETPOINT', d.sp_hz + ' Hz' + (d.sp_halted ? ' (DA DUNG)' : ''),
                  (d.sp_halted || d.sp_hz < 18) ? 'warn' : '');
        h += cell('WATCHDOG', d.wd_trips + ' / ' + d.loop_wd,
                  (d.wd_trips || d.loop_wd) ? 'warn' : '');
        h += cell('FPS / INFER', d.fps + ' / ' + d.infer_ms + ' ms');
        document.getElementById('grid').innerHTML = h;
        document.getElementById('raw').innerText = JSON.stringify(d, null, 1);
    }).catch(e => {});
}, 250);
</script>
</head>
<body>
    <div id="banner">Dang ket noi...</div>
    <div><img id="video" src="/stream.mjpg?t={TOKEN}" onmousedown="pickTarget(event)" draggable="false"></div>
    <div>
        <button id="engage" class="btn" onpointerdown="startEngage()"
                onpointerup="stopEngage()" onpointerleave="stopEngage()"
                onpointercancel="stopEngage()">ENGAGE (GIU)</button>
        <button id="disengage" class="btn" onclick="stopEngage()">DISENGAGE</button>
        <button id="unlock" class="btn" onclick="post('/unlock')">BO KHOA</button>
    </div>
    <div id="grid"></div>
    <div id="raw"></div>
</body>
</html>
"""


def start_web(port, op_link, get_stats, get_jpeg, cmd_queue,
              bind_host="127.0.0.1"):
    class RequestHandler(http.server.BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def _deny(self, code=403, body=b"Forbidden"):
            self.send_response(code)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _check_auth(self, qs):
            if not op_link.authed(qs):
                self._deny()
                return False
            return True

        def _check_origin(self):
            """Chan request tu trang la (CSRF) - chi cho cung host."""
            org = self.headers.get('Origin') or self.headers.get('Referer')
            if not org:
                return True                     # fetch cung goc co the khong gui
            host = self.headers.get('Host', '')
            # Khong dung `host in org`: https://ke-xau/127.0.0.1:8080 se
            # qua duoc phep. So sanh dung authority cua URL.
            try:
                origin_host = urllib.parse.urlparse(org).netloc
            except ValueError:
                origin_host = ''
            if host and origin_host == host:
                return True
            self._deny(403, b"Bad origin")
            return False

        def _ok(self, body=b"", ctype="text/plain"):
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            if body:
                self.wfile.write(body)

        # ------------------------------------------------------------ GET
        def do_GET(self):
            parsed = urllib.parse.urlparse(self.path)
            qs = urllib.parse.parse_qs(parsed.query)

            if parsed.path == '/':
                # Trang goc co chua token trong JavaScript. Neu khong bao ve
                # no thi bat ky ai vao duoc port deu lay token va dieu khien.
                if not self._check_auth(qs):
                    return
                html = HTML_TEMPLATE.replace('{TOKEN}', op_link.token)
                self._ok(html.encode('utf-8'), "text/html; charset=utf-8")

            elif parsed.path == '/stream.mjpg':
                if not self._check_auth(qs):
                    return
                self.send_response(200)
                self.send_header('Age', '0')
                self.send_header('Cache-Control', 'no-cache, private')
                self.send_header('Pragma', 'no-cache')
                self.send_header(
                    'Content-Type',
                    'multipart/x-mixed-replace; boundary=FRAME')
                self.end_headers()
                self.close_connection = True
                last = None
                try:
                    while True:
                        jpeg = get_jpeg()
                        if jpeg and jpeg is not last:
                            last = jpeg
                            self.wfile.write(
                                b'--FRAME\r\nContent-Type: image/jpeg\r\n'
                                b'Content-Length: ' +
                                str(len(jpeg)).encode() + b'\r\n\r\n')
                            self.wfile.write(jpeg)
                            self.wfile.write(b'\r\n')
                        time.sleep(0.04)
                except Exception:
                    pass

            elif parsed.path == '/stats':
                if not self._check_auth(qs):
                    return
                self._ok(json.dumps(get_stats()).encode('utf-8'),
                         "application/json")
            else:
                self._deny(404, b"Not found")

        # ----------------------------------------------------------- POST
        def do_POST(self):
            parsed = urllib.parse.urlparse(self.path)
            qs = urllib.parse.parse_qs(parsed.query)

            # DISENGAGE phai chay ke ca khi kiem tra khac that bai
            if parsed.path == '/disengage':
                if not self._check_auth(qs):
                    return
                op_link.request_stop()
                self._ok()
                return

            if not self._check_auth(qs):
                return
            if not self._check_origin():
                return

            if parsed.path == '/alive':
                op_link.beat()
                self._ok()
            elif parsed.path == '/engage':
                try:
                    cmd_queue.put_nowait({'type': 'engage'})
                except Exception:
                    pass
                self._ok()
            elif parsed.path == '/pick':
                if 'x' in qs and 'y' in qs:
                    try:
                        cmd_queue.put_nowait({
                            'type': 'pick',
                            'x': float(qs['x'][0]),
                            'y': float(qs['y'][0])})
                    except Exception:
                        pass
                self._ok()
            elif parsed.path == '/unlock':
                try:
                    cmd_queue.put_nowait({'type': 'unlock'})
                except Exception:
                    pass
                self._ok()
            else:
                self._deny(404, b"Not found")

        def log_message(self, fmt, *a):
            pass    # Tat log truy cap de khong lam roi console

    server = http.server.ThreadingHTTPServer((bind_host, port), RequestHandler)
    server.daemon_threads = True
    t = threading.Thread(target=server.serve_forever, daemon=True,
                         name="webui")
    t.start()
    return server
