"""
Raspberry Pi 3 RC Car Web Dashboard.
  - Live Camera Feed (640x480)
  - Servo Pan Slider (GPIO 12, jitter-free Hardware PWM)
  - Touch D-Pad (discrete commands)
  - Proportional Analog Joystick (full tank-mix, minute inputs supported)
  - Photo Capture & Video Recording
"""

import os
import sys
import threading
from typing import Optional, Dict, Any
from flask import Flask, Response, jsonify, request, render_template_string, send_file

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from config import DashboardConfig
from hardware.motors import MotorController
from hardware.servo import ServoController
from vision.camera import Camera


class DashboardServer:
    def __init__(
        self,
        config: DashboardConfig,
        motor_controller: Optional[MotorController] = None,
        servo_controller: Optional[ServoController] = None,
        camera: Optional[Camera] = None,
    ) -> None:
        self.config  = config
        self.motors  = motor_controller or MotorController()
        self.servo   = servo_controller or ServoController()
        self.camera  = camera or Camera()
        self.app     = Flask(__name__)

        self._telem: Dict[str, Any] = {
            "status":       "STOPPED",
            "servo_angle":  90.0,
            "x":            0.0,
            "y":            0.0,
            "is_recording": False,
        }
        self._lock = threading.Lock()
        self._setup_routes()

    # ------------------------------------------------------------------
    def _setup_routes(self) -> None:

        @self.app.route("/")
        def index():
            return render_template_string(DASHBOARD_HTML)

        @self.app.route("/api/telemetry")
        def api_telemetry():
            with self._lock:
                return jsonify(self._telem)

        @self.app.route("/api/frame")
        def api_frame():
            if self.camera and self.camera.is_initialized:
                frame = self.camera.get_frame()
                if frame:
                    r = Response(frame, mimetype="image/jpeg")
                    r.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
                    r.headers["Pragma"]        = "no-cache"
                    r.headers["Expires"]       = "0"
                    return r
            return Response(b"", mimetype="image/jpeg", status=204)

        @self.app.route("/video_feed")
        def video_feed():
            if self.camera and self.camera.is_initialized:
                return Response(
                    self.camera.generate_mjpeg(),
                    mimetype="multipart/x-mixed-replace; boundary=frame"
                )
            return Response(b"", status=204)

        # ---- SERVO ---------------------------------------------------
        @self.app.route("/api/servo", methods=["POST"])
        def api_servo():
            data  = request.json or {}
            angle = float(data.get("angle", 90.0))
            self.servo.set_angle(angle)
            with self._lock:
                self._telem["servo_angle"] = round(angle, 1)
            return jsonify({"status": "success", "angle": angle})

        # ---- DISCRETE MOTOR ------------------------------------------
        @self.app.route("/api/motor/<cmd>", methods=["POST"])
        def api_motor_cmd(cmd: str):
            cmd = cmd.lower()
            if   cmd == "forward":  self.motors.forward();  status = "FORWARD"
            elif cmd == "backward": self.motors.backward(); status = "REVERSE"
            elif cmd == "left":     self.motors.left();     status = "TURN-LEFT"
            elif cmd == "right":    self.motors.right();    status = "TURN-RIGHT"
            else:                   self.motors.stop();     status = "STOPPED"
            with self._lock:
                self._telem["status"] = status
            return jsonify({"status": "success", "command": cmd, "motor_state": status})

        # ---- PROPORTIONAL JOYSTICK -----------------------------------
        @self.app.route("/api/joystick", methods=["POST"])
        def api_joystick():
            data = request.json or {}
            x = float(data.get("x", 0.0))
            y = float(data.get("y", 0.0))
            with self._lock:
                self._telem["x"] = round(x, 2)
                self._telem["y"] = round(y, 2)
            status = self.motors.drive(throttle=y, steering=x)
            with self._lock:
                self._telem["status"] = status
            return jsonify({"status": "success", "motor_state": status})

        # ---- PHOTO CAPTURE ------------------------------------------
        @self.app.route("/api/capture/photo", methods=["POST"])
        def api_capture_photo():
            filename = self.camera.capture_photo()
            if filename:
                return jsonify({"status": "success", "filename": filename})
            return jsonify({"status": "error", "message": "Camera not ready"}), 503

        # ---- VIDEO RECORDING ----------------------------------------
        @self.app.route("/api/capture/video/start", methods=["POST"])
        def api_video_start():
            if self.camera.is_recording:
                return jsonify({"status": "already_recording"})
            filename = self.camera.start_recording()
            if filename:
                with self._lock:
                    self._telem["is_recording"] = True
                return jsonify({"status": "recording", "filename": filename})
            return jsonify({"status": "error", "message": "Cannot start recording"}), 503

        @self.app.route("/api/capture/video/stop", methods=["POST"])
        def api_video_stop():
            filename = self.camera.stop_recording()
            with self._lock:
                self._telem["is_recording"] = False
            if filename:
                return jsonify({"status": "saved", "filename": filename})
            return jsonify({"status": "not_recording"})

        # ---- MEDIA LIST ---------------------------------------------
        @self.app.route("/api/media/list")
        def api_media_list():
            return jsonify(self.camera.list_media())

        # ---- MEDIA FILE DOWNLOAD ------------------------------------
        @self.app.route("/api/media/<path:filename>")
        def api_media_file(filename):
            from vision.camera import MEDIA_DIR
            safe = os.path.basename(filename)
            path = os.path.join(MEDIA_DIR, safe)
            if not os.path.exists(path):
                return jsonify({"error": "not found"}), 404
            mime = "image/jpeg" if safe.endswith(".jpg") else "video/x-motion-jpeg"
            return send_file(path, mimetype=mime, as_attachment=True)

    # ------------------------------------------------------------------
    def start(self) -> None:
        print("=" * 71)
        print(" RASPBERRY PI 3 - CAMERA + MOTOR (PWM) + SERVO (HW-PWM) DASHBOARD")
        print("=" * 71)
        self.app.run(
            host=self.config.host,
            port=self.config.port,
            debug=False,
            use_reloader=False,
            threaded=True,
        )


# ======================================================================
#  DASHBOARD HTML
# ======================================================================
DASHBOARD_HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>RC Car</title>
<style>
/* ─────────────────────────────────────────────────────────────
   RESET
───────────────────────────────────────────────────────────── */
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0;
  user-select:none;-webkit-user-select:none;-webkit-tap-highlight-color:transparent;}

/* ─────────────────────────────────────────────────────────────
   TOKENS — Apple monochrome system
───────────────────────────────────────────────────────────── */
:root{
  --bg      : #F2F2F7;
  --s1      : #FFFFFF;
  --s2      : #F9F9FB;
  --card    : #FFFFFF;
  --card2   : #F8F8FA;
  --bdr     : #E5E5EA;
  --bdr2    : #D1D1D6;
  --txt     : #1C1C1E;
  --txt2    : #636366;
  --txt3    : #8E8E93;
  --red     : #FF3B30;   /* Apple red    */
  --grn     : #34C759;   /* Apple green  */
  --r       : 20px;
  --rf      : -apple-system,BlinkMacSystemFont,'SF Pro Display','Helvetica Neue',Arial,sans-serif;
}

/* ─────────────────────────────────────────────────────────────
   BASE
───────────────────────────────────────────────────────────── */
html,body{height:100%;overscroll-behavior:none;}
body{
  background:var(--bg);
  color:var(--txt);
  font-family:var(--rf);
  min-height:100vh;
  touch-action:pan-y;
  overflow-x:hidden;
  /* Apple dot-grid pattern */
  background-image:
    radial-gradient(circle,rgba(0,0,0,.065) 1px,transparent 1px);
  background-size:28px 28px;
  background-attachment:fixed;
}

/* ─────────────────────────────────────────────────────────────
   FROSTED CARD
───────────────────────────────────────────────────────────── */
.card{
  background:var(--card);
  border-radius:var(--r);
  border:1px solid var(--bdr);
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.05);
  position:relative;overflow:hidden;
}
.card::before{
  content:'';position:absolute;inset:0;border-radius:var(--r);
  background:linear-gradient(145deg,rgba(255,255,255,.9) 0%,transparent 50%);
  pointer-events:none;
}

/* Section label */
.lbl{
  font-size:10px;font-weight:700;letter-spacing:.12em;
  text-transform:uppercase;color:var(--txt2);
  margin-bottom:10px;display:flex;align-items:center;gap:6px;
}
.lbl-dot{width:5px;height:5px;border-radius:50%;background:var(--txt2);flex-shrink:0;}

/* ─────────────────────────────────────────────────────────────
   STATUS BAR (top)
───────────────────────────────────────────────────────────── */
.topbar{
  display:flex;align-items:center;justify-content:space-between;
  padding:14px 16px 10px;
  max-width:1240px;margin:0 auto;
}
.topbar-left{display:flex;align-items:center;gap:10px;}
.app-icon{
  width:32px;height:32px;border-radius:8px;
  background:#1C1C1E;color:#fff;display:flex;align-items:center;justify-content:center;
  font-size:.9rem;flex-shrink:0;
}
.app-name{font-size:1rem;font-weight:700;letter-spacing:-.02em;color:#1C1C1E;}
.app-sub{font-size:10px;color:var(--txt2);margin-top:1px;font-weight:500;}
.live-pill{
  display:flex;align-items:center;gap:5px;
  background:rgba(52,199,89,.12);border:1px solid rgba(52,199,89,.3);
  border-radius:20px;padding:5px 10px;
  font-size:10px;font-weight:700;letter-spacing:.06em;color:#1E7E34;
}
.live-dot{width:6px;height:6px;border-radius:50%;
  background:var(--grn);box-shadow:0 0 6px var(--grn);animation:blink 2s infinite;}
@keyframes blink{0%,100%{opacity:1;}50%{opacity:.3;}}

/* ─────────────────────────────────────────────────────────────
   DESKTOP GRID (FULL SCREEN SPACE UTILIZATION)
───────────────────────────────────────────────────────────── */
.layout{
  display:grid;
  grid-template-columns:1.4fr 1fr;
  grid-template-rows:auto auto auto;
  gap:16px;
  padding:0 16px 28px;
  max-width:1240px;margin:0 auto;
}
.a-cam   {grid-column:1;grid-row:1/3;}
.a-stat  {grid-column:2;grid-row:1;}
.a-servo {grid-column:2;grid-row:2;}
.a-ctrl  {grid-column:1/-1;grid-row:3;}

/* ─────────────────────────────────────────────────────────────
   MOBILE OVERRIDES (EXACT ORDER: CAM -> JOYSTICK -> ACTIONS -> SERVO -> GALLERY)
───────────────────────────────────────────────────────────── */
@media(max-width:680px){
  .layout{
    display: flex;
    flex-direction: column;
    padding: 0 12px 32px;
    gap: 12px;
  }
  .a-cam       { order: 1; }
  .a-ctrl      { order: 2; }
  .a-stat      { display: none !important; }
  .a-servo     { display: none !important; }
  .mobile-dock { order: 3; display: flex; }
}

/* ─────────────────────────────────────────────────────────────
   CAMERA CARD
───────────────────────────────────────────────────────────── */
.a-cam{padding:10px;}
.cam-frame{
  border-radius:14px;overflow:hidden;
  background:#0a0a0a;border:1px solid rgba(255,255,255,.12);
  position:relative;
}
.cam-frame img{
  width:100%;aspect-ratio:4/3;object-fit:cover;display:block;
}
.cam-hud{
  position:absolute;bottom:0;left:0;right:0;
  background:linear-gradient(transparent,rgba(0,0,0,.7));
  display:flex;align-items:flex-end;justify-content:space-between;
  padding:20px 12px 10px;
}
.cam-badge{
  display:flex;align-items:center;gap:4px;
  font-size:10px;font-weight:700;letter-spacing:.07em;color:rgba(255,255,255,.8);
}
.rec-dot{width:6px;height:6px;border-radius:50%;background:var(--red);animation:blink 1s infinite;}
.cam-res{font-size:10px;color:rgba(255,255,255,.45);font-weight:600;}

/* ─────────────────────────────────────────────────────────────
   STATUS CARD (desktop only)
───────────────────────────────────────────────────────────── */
.a-stat{padding:16px;}
.st-wrap{display:flex;flex-direction:column;align-items:center;gap:10px;}
.st-ring{position:relative;width:80px;height:80px;}
.st-ring svg{width:80px;height:80px;transform:rotate(-90deg);}
.ring-bg{fill:none;stroke:rgba(255,255,255,.08);stroke-width:5;}
.ring-arc{fill:none;stroke:#fff;stroke-width:5;stroke-linecap:round;
  stroke-dasharray:215;stroke-dashoffset:60;
  transition:stroke .35s,stroke-dashoffset .4s;}
.st-ico{
  position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);
  font-size:1.3rem;
}
.st-label{font-size:1rem;font-weight:700;letter-spacing:-.01em;}
.st-sub{font-size:10px;color:var(--txt2);margin-top:1px;}
.hud-row{display:grid;grid-template-columns:1fr 1fr;gap:6px;width:100%;}
.hud-box{
  background:#1C1C1E;border:1px solid #1C1C1E;
  border-radius:10px;padding:7px 9px;color:#fff;
}
.hud-k{font-size:9px;text-transform:uppercase;letter-spacing:.1em;
  color:rgba(255,255,255,.6);font-weight:700;margin-bottom:2px;}
.hud-v{font-size:.85rem;font-weight:800;font-variant-numeric:tabular-nums;color:#fff;}

/* ─────────────────────────────────────────────────────────────
   SERVO CARD (desktop only)
───────────────────────────────────────────────────────────── */
.a-servo{padding:16px;}
.sv-top{display:flex;align-items:center;gap:12px;margin-bottom:10px;}
.sv-num{font-size:2.2rem;font-weight:800;letter-spacing:-.04em;line-height:1;color:#1C1C1E;}
.sv-num sup{font-size:.85rem;font-weight:500;color:var(--txt2);}
.sv-lbl{font-size:10px;color:var(--txt2);font-weight:600;margin-top:3px;}
input[type=range]{
  -webkit-appearance:none;appearance:none;
  width:100%;height:4px;border-radius:2px;
  background:linear-gradient(to right,#1C1C1E var(--pct,50%),#E5E5EA var(--pct,50%));
  outline:none;cursor:pointer;margin-bottom:10px;
}
input[type=range]::-webkit-slider-thumb{
  -webkit-appearance:none;width:18px;height:18px;border-radius:50%;
  background:#fff;border:2px solid #1C1C1E;
  box-shadow:0 2px 8px rgba(0,0,0,.2);
  cursor:grab;transition:transform .1s;
}
input[type=range]::-webkit-slider-thumb:active{transform:scale(1.2);}
input[type=range]::-moz-range-thumb{
  width:18px;height:18px;border-radius:50%;
  background:#fff;border:2px solid #1C1C1E;cursor:grab;
}
.sv-btns{display:flex;gap:6px;}
.sv-btn{
  flex:1;height:32px;font-size:11px;font-weight:700;font-family:var(--rf);
  background:#1C1C1E;color:#fff;
  border:1px solid #1C1C1E;border-radius:8px;cursor:pointer;
  transition:background .1s;
}
.sv-btn:hover{background:#3A3A3C;}
.sv-btn:active{background:#545456;}

/* ─────────────────────────────────────────────────────────────
   CONTROLS ROW (bottom)
───────────────────────────────────────────────────────────── */
.a-ctrl{
  display:grid;
  grid-template-columns:1fr auto 1fr;
  gap:12px;align-items:start;
  padding:16px;
}
@media(max-width:680px){
  .a-ctrl{
    grid-template-columns:1fr;
    gap:12px;
  }
}

/* D-PAD (desktop only) */
.dpad-section{display:flex;flex-direction:column;gap:10px;}
.dpad-wrap{display:flex;justify-content:center;}
.dpad{
  display:grid;
  grid-template-columns:repeat(3,56px);
  grid-template-rows:repeat(3,56px);
  gap:7px;
}
.d-btn{
  border-radius:13px;
  background:#FFFFFF;
  border:1px solid #E5E5EA;
  color:#1C1C1E;font-size:1.15rem;
  display:flex;align-items:center;justify-content:center;
  cursor:pointer;
  transition:background .06s,transform .05s,box-shadow .06s;
  touch-action:manipulation;font-family:var(--rf);
  box-shadow:0 2px 8px rgba(0,0,0,.04);
}
.d-btn:active{background:#1C1C1E;color:#FFFFFF;transform:scale(.9);}
.d-stp{background:rgba(255,59,48,.15);border-color:rgba(255,59,48,.4);color:var(--red);}
.d-stp:active{background:var(--red);color:#fff;}

/* JOYSTICK */
.joy-section{display:flex;flex-direction:column;align-items:center;gap:10px;}
#jc{display:block;touch-action:none;cursor:grab;}
#jc:active{cursor:grabbing;}
.joy-meta{display:flex;gap:7px;align-items:center;flex-wrap:wrap;justify-content:center;}
.joy-tag{
  background:#1C1C1E;border:1px solid #1C1C1E;
  border-radius:7px;padding:3px 9px;
  font-size:10px;color:rgba(255,255,255,.7);font-weight:600;font-variant-numeric:tabular-nums;
}
.joy-tag b{color:#FFFFFF;}

/* CAPTURE column */
.cap-section{display:flex;flex-direction:column;gap:8px;}

/* ─────────────────────────────────────────────────────────────
   MOBILE BOTTOM DOCK
   On mobile, stop + photo + video shown BELOW joystick
───────────────────────────────────────────────────────────── */
.mobile-dock{
  display:none;
  flex-direction:column;
  gap:10px;
  padding:0 12px 8px;
  max-width:900px;margin:0 auto;
}
@media(max-width:680px){
  .mobile-dock{display:flex;}
  /* hide desktop capture section inside .a-ctrl on mobile */
  .cap-section{display:none;}
  /* joystick gets bigger on mobile */
}

/* ─────────────────────────────────────────────────────────────
   APPLE-STYLE BUTTONS
───────────────────────────────────────────────────────────── */
/* Stop — large pill, always red */
.btn-stop{
  width:100%;height:58px;
  border-radius:100px;
  background:var(--red);
  border:none;
  font-size:1rem;font-weight:700;letter-spacing:.01em;
  color:#fff;font-family:var(--rf);cursor:pointer;
  display:flex;align-items:center;justify-content:center;gap:8px;
  transition:opacity .1s,transform .08s;
  box-shadow:0 0 0 1px rgba(255,59,48,.5),0 4px 20px rgba(255,59,48,.3);
}
.btn-stop:active{opacity:.8;transform:scale(.97);}

/* Photo + Video — pill style side by side */
.cap-row{display:grid;grid-template-columns:1fr 1fr;gap:8px;}

.btn-cap{
  height:52px;
  border-radius:14px;
  font-size:.88rem;font-weight:700;letter-spacing:.01em;
  font-family:var(--rf);cursor:pointer;
  display:flex;align-items:center;justify-content:center;gap:7px;
  transition:background .1s,transform .07s,box-shadow .1s;
  border:1px solid;
}
.btn-cap:active{transform:scale(.95);}

.btn-photo{
  background:#1C1C1E;
  border-color:#1C1C1E;
  color:#fff;
}
.btn-photo:hover{background:rgba(255,255,255,.12);}
.btn-photo:active{background:rgba(255,255,255,.20);}

.btn-video{
  background:rgba(255,67,58,.08);
  border-color:rgba(255,67,58,.25);
  color:var(--red);
}
.btn-video.recording{
  background:rgba(255,67,58,.16);
  border-color:var(--red);
  box-shadow:0 0 0 2px rgba(255,67,58,.2);
}
.btn-video:active{background:rgba(255,67,58,.28);}

/* Recording timer badge */
.rec-bar{
  display:none;
  align-items:center;justify-content:center;gap:7px;
  height:36px;border-radius:10px;
  background:rgba(255,67,58,.08);border:1px solid rgba(255,67,58,.2);
  font-size:12px;font-weight:700;color:var(--red);letter-spacing:.04em;
}
.rec-bar.on{display:flex;}
.rec-bar-dot{width:7px;height:7px;border-radius:50%;background:var(--red);animation:blink .7s infinite;}

/* Capture status message */
.cap-msg{
  text-align:center;font-size:11px;color:var(--txt2);font-weight:500;
  min-height:16px;transition:color .2s;
}
.cap-msg.ok{color:var(--grn);}

/* ─────────────────────────────────────────────────────────────
   GALLERY (desktop only)
───────────────────────────────────────────────────────────── */
.gallery-wrap{margin-top:4px;}
.gallery-hd{
  font-size:10px;text-transform:uppercase;letter-spacing:.1em;
  color:var(--txt2);font-weight:700;margin-bottom:8px;
}
.gallery{
  display:grid;
  grid-template-columns:repeat(auto-fill,minmax(80px,1fr));
  gap:6px;
}
.gitem{
  border-radius:10px;overflow:hidden;border:1px solid var(--bdr);
  background:#0a0a0a;position:relative;aspect-ratio:4/3;cursor:pointer;
  transition:transform .12s,border-color .12s;
}
.gitem:hover{transform:scale(1.04);border-color:rgba(255,255,255,.28);}
.gitem img{width:100%;height:100%;object-fit:cover;display:block;}
.gitem-t{
  position:absolute;top:4px;left:4px;
  background:rgba(0,0,0,.65);border-radius:4px;
  font-size:8px;font-weight:800;color:#fff;padding:1px 4px;letter-spacing:.05em;
}
.gitem-vid{display:flex;align-items:center;justify-content:center;
  width:100%;height:100%;font-size:1.6rem;}
.g-dl{
  position:absolute;inset:0;background:rgba(0,0,0,.5);
  display:none;align-items:center;justify-content:center;
  font-size:1.2rem;color:#fff;text-decoration:none;
}
.gitem:hover .g-dl{display:flex;}
.g-empty{
  grid-column:1/-1;text-align:center;color:var(--txt3);
  font-size:11px;padding:16px 0;
}

/* Flash overlay */
@keyframes flash{0%{opacity:.6}100%{opacity:0}}
.flash{
  position:fixed;inset:0;background:#fff;
  pointer-events:none;opacity:0;
  animation:flash .3s ease-out forwards;
  z-index:999;
}

/* ─────────────────────────────────────────────────────────────
   DESKTOP EXTRA PANEL row separator
───────────────────────────────────────────────────────────── */
.sep{
  height:1px;background:rgba(255,255,255,.06);
  grid-column:1/-1;margin:2px 0;
}
</style>
</head>
<body>

<!-- ── STATUS BAR ── -->
<div class="topbar">
  <div class="topbar-left">
    <div class="app-icon">🚗</div>
    <div>
      <div class="app-name">RC Car</div>
      <div class="app-sub">Raspberry Pi 3 &bull; Dashboard</div>
    </div>
  </div>
  <div class="live-pill">
    <div class="live-dot"></div>
    CONNECTED
  </div>
</div>

<!-- ── MAIN LAYOUT ── -->
<div class="layout">

  <!-- 1. CAMERA -->
  <div class="card a-cam">
    <div class="lbl desktop-only"><div class="lbl-dot"></div>Live Feed</div>
    <div class="cam-frame">
      <img id="cam" src="/api/frame" alt="live">
      <div class="cam-hud">
        <div class="cam-badge"><div class="rec-dot"></div>LIVE</div>
        <span class="cam-res">640 &times; 480</span>
      </div>
    </div>
  </div>

  <!-- STATUS (desktop only) -->
  <div class="card a-stat desktop-only">
    <div class="lbl"><div class="lbl-dot"></div>Status</div>
    <div class="st-wrap">
      <div class="st-ring">
        <svg viewBox="0 0 80 80">
          <circle class="ring-bg" cx="40" cy="40" r="34"/>
          <circle id="ring" class="ring-arc" cx="40" cy="40" r="34"/>
        </svg>
        <div class="st-ico" id="st-ico">&#9209;</div>
      </div>
      <div style="text-align:center">
        <div class="st-label" id="st-name">STOPPED</div>
        <div class="st-sub">Motor State</div>
      </div>
      <div class="hud-row">
        <div class="hud-box"><div class="hud-k">Joy X</div><div class="hud-v" id="hx">0.00</div></div>
        <div class="hud-box"><div class="hud-k">Joy Y</div><div class="hud-v" id="hy">0.00</div></div>
        <div class="hud-box"><div class="hud-k">Servo</div><div class="hud-v"><span id="hs">90</span>°</div></div>
        <div class="hud-box"><div class="hud-k">Signal</div><div class="hud-v" style="color:var(--grn)">LIVE</div></div>
      </div>
    </div>
  </div>

  <!-- SERVO (desktop only) -->
  <div class="card a-servo desktop-only">
    <div class="lbl"><div class="lbl-dot"></div>Servo Pan &mdash; GPIO 12</div>
    <div class="sv-top">
      <svg width="60" height="38" viewBox="0 0 60 38" style="flex-shrink:0">
        <path d="M4 36 A27 27 0 0 1 56 36" fill="none" stroke="rgba(255,255,255,.1)" stroke-width="4" stroke-linecap="round"/>
        <path id="arc" d="M4 36 A27 27 0 0 1 56 36" fill="none" stroke="#fff" stroke-width="4" stroke-linecap="round"
              stroke-dasharray="85" stroke-dashoffset="42" style="transition:stroke-dashoffset .2s"/>
        <defs><filter id="gf"><feGaussianBlur stdDeviation="2" result="b"/>
          <feMerge><feMergeNode in="b"/><feMergeNode in="SourceGraphic"/></feMerge></filter></defs>
        <circle id="aknob" cx="30" cy="9" r="4.5" fill="#fff" filter="url(#gf)" style="transition:cx .2s,cy .2s"/>
      </svg>
      <div>
        <div class="sv-num"><span id="sdeg">90</span><sup>°</sup></div>
        <div class="sv-lbl">Pan Angle</div>
      </div>
    </div>
    <input type="range" id="sl" min="0" max="180" value="90"
           oninput="setServo(this.value)" style="--pct:50%">
    <div class="sv-btns">
      <button class="sv-btn" onclick="setServo(0)">&#9664; 0°</button>
      <button class="sv-btn" onclick="setServo(90)">Center</button>
      <button class="sv-btn" onclick="setServo(180)">180° &#9654;</button>
    </div>
  </div>

  <!-- CONTROLS ROW -->
  <div class="card a-ctrl">

    <!-- D-Pad (desktop only) -->
    <div class="dpad-section desktop-only">
      <div class="lbl"><div class="lbl-dot"></div>D-Pad</div>
      <div class="dpad-wrap">
        <div class="dpad">
          <div></div>
          <button class="d-btn" onpointerdown="hold('forward')"  onpointerup="rel()" onpointercancel="rel()">&#9650;</button>
          <div></div>
          <button class="d-btn" onpointerdown="hold('left')"     onpointerup="rel()" onpointercancel="rel()">&#9664;</button>
          <button class="d-btn d-stp" onpointerdown="rel()">&#9632;</button>
          <button class="d-btn" onpointerdown="hold('right')"    onpointerup="rel()" onpointercancel="rel()">&#9654;</button>
          <div></div>
          <button class="d-btn" onpointerdown="hold('backward')" onpointerup="rel()" onpointercancel="rel()">&#9660;</button>
          <div></div>
        </div>
      </div>
      <!-- Desktop capture buttons under dpad -->
      <div class="cap-row" style="margin-top:8px;">
        <button class="btn-cap btn-photo" onclick="capturePhoto()">&#128248; Photo</button>
        <button class="btn-cap btn-video" id="btn-vid" onclick="toggleVideo()">
          <span id="vid-ico">&#9654;</span>&nbsp;<span id="vid-lbl">Record</span>
        </button>
      </div>
      <div class="rec-bar" id="rec-bar"><div class="rec-bar-dot"></div><span id="rec-t">00:00</span></div>
      <div class="cap-msg" id="cap-msg">Ready</div>
      <div class="gallery-wrap">
        <div class="gallery-hd">&#128247; Gallery</div>
        <div class="gallery" id="gallery"><div class="g-empty">No captures yet</div></div>
      </div>
    </div>

    <!-- 2. JOYSTICK (center) -->
    <div class="joy-section">
      <div class="lbl"><div class="lbl-dot"></div>Analog Controller</div>
      <canvas id="jc" width="240" height="240"></canvas>
      <div class="joy-meta">
        <div class="joy-tag">X:&nbsp;<b id="vx">0.00</b></div>
        <div class="joy-tag">Y:&nbsp;<b id="vy">0.00</b></div>
        <div class="joy-tag" id="st-badge">STOPPED</div>
      </div>
    </div>

    <!-- CAPTURE (desktop right column) -->
    <div class="cap-section desktop-only">
      <div class="lbl"><div class="lbl-dot"></div>Capture</div>
      <button class="btn-stop" onclick="rel()">&#9632;&ensp;STOP</button>
      <div style="height:4px"></div>
      <div class="cap-row">
        <button class="btn-cap btn-photo" onclick="capturePhoto()">&#128248; Photo</button>
        <button class="btn-cap btn-video" id="btn-vid2" onclick="toggleVideo()">
          <span id="vid-ico2">&#9654;</span>&nbsp;<span id="vid-lbl2">Record</span>
        </button>
      </div>
      <div class="rec-bar" id="rec-bar2"><div class="rec-bar-dot"></div><span id="rec-t2">00:00</span></div>
      <div class="cap-msg" id="cap-msg2">Ready</div>
    </div>

  </div><!-- /a-ctrl -->

</div><!-- /layout -->

<!-- ── MOBILE DOCK (below joystick on mobile) ── -->
<div class="mobile-dock">
  <!-- 1. STOP -->
  <button class="btn-stop" onclick="rel()">&#9632;&ensp;STOP</button>

  <!-- 2. Photo + Video Buttons -->
  <div class="cap-row">
    <button class="btn-cap btn-photo" onclick="capturePhoto()">&#128248;&nbsp;Photo</button>
    <button class="btn-cap btn-video" id="btn-vid-m" onclick="toggleVideo()">
      <span id="vid-ico-m">&#9654;</span>&nbsp;<span id="vid-lbl-m">Record</span>
    </button>
  </div>
  <div class="rec-bar" id="rec-bar-m"><div class="rec-bar-dot"></div><span id="rec-t-m">00:00</span></div>
  <div class="cap-msg" id="cap-msg-m">Ready</div>

  <!-- 3. Mobile Servo Pan Mode Control Panel -->
  <div class="card" style="padding:14px;margin-top:4px;">
    <div class="lbl"><div class="lbl-dot"></div>Servo Pan Mode &mdash; GPIO 12</div>
    <div class="sv-top">
      <div>
        <div class="sv-num"><span id="sdeg-m">90</span><sup>°</sup></div>
        <div class="sv-lbl">Pan Angle</div>
      </div>
    </div>
    <input type="range" id="sl-m" min="0" max="180" value="90"
           oninput="setServo(this.value)" style="--pct:50%">
    <div class="sv-btns">
      <button class="sv-btn" onclick="setServo(0)">0°</button>
      <button class="sv-btn" onclick="setServo(45)">45°</button>
      <button class="sv-btn" onclick="setServo(90)">Center</button>
      <button class="sv-btn" onclick="setServo(135)">135°</button>
      <button class="sv-btn" onclick="setServo(180)">180°</button>
    </div>
  </div>

  <!-- 4. Mobile Saved Media Gallery (below Servo Pan mode section) -->
  <div class="card" style="padding:14px;margin-top:4px;">
    <div class="gallery-hd">&#128247; Saved Media Gallery</div>
    <div class="gallery" id="gallery-m"><div class="g-empty">No captures yet</div></div>
  </div>
</div>

<script>
/* ── CAMERA ──────────────────────────────────────────── */
(function(){
  var img=document.getElementById('cam'),busy=false;
  setInterval(function(){
    if(busy)return;busy=true;
    var n=new Image();
    n.onload=function(){img.src=n.src;busy=false;};
    n.onerror=function(){busy=false;};
    n.src='/api/frame?_='+Date.now();
  },42);
}());

/* ── SERVO ───────────────────────────────────────────── */
function setServo(v){
  v=Math.round(+v);
  var sd=document.getElementById('sdeg'); if(sd)sd.textContent=v;
  var sdm=document.getElementById('sdeg-m'); if(sdm)sdm.textContent=v;
  var hs=document.getElementById('hs'); if(hs)hs.textContent=v;
  var sl=document.getElementById('sl');
  if(sl){sl.value=v;sl.style.setProperty('--pct',(v/180*100)+'%');}
  var slm=document.getElementById('sl-m');
  if(slm){slm.value=v;slm.style.setProperty('--pct',(v/180*100)+'%');}
  _arc(v);
  fetch('/api/servo',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({angle:v})});
}
function _arc(d){
  var off=85-(d/180)*85;
  var el=document.getElementById('arc');
  if(el)el.style.strokeDashoffset=off;
  var a=(1-d/180)*Math.PI;
  var kx=30+27*Math.cos(Math.PI-a),ky=36-27*Math.sin(a);
  var k=document.getElementById('aknob');
  if(k){k.setAttribute('cx',kx.toFixed(1));k.setAttribute('cy',ky.toFixed(1));}
}
_arc(90);

/* ── STATUS RING ─────────────────────────────────────── */
var SCFG={
  'STOPPED'       :{ico:'&#9209;', off:185},
  'FORWARD'       :{ico:'&#11014;',off:25},
  'REVERSE'       :{ico:'&#11015;',off:145},
  'TURN-LEFT'     :{ico:'&#8624;', off:75},
  'TURN-RIGHT'    :{ico:'&#8625;', off:75},
  'FORWARD-LEFT'  :{ico:'&#8598;', off:50},
  'FORWARD-RIGHT' :{ico:'&#8599;', off:50},
  'REVERSE-LEFT'  :{ico:'&#8601;', off:148},
  'REVERSE-RIGHT' :{ico:'&#8600;', off:148},
};
var _lst='';
function setStatus(st){
  if(st===_lst)return;_lst=st;
  var c=SCFG[st]||SCFG['STOPPED'];
  var ico=document.getElementById('st-ico');
  if(ico)ico.innerHTML=c.ico;
  var nm=document.getElementById('st-name');
  if(nm)nm.textContent=st.replace(/-/g,' ');
  var r=document.getElementById('ring');
  if(r)r.style.strokeDashoffset=c.off;
  var b=document.getElementById('st-badge');
  if(b)b.innerHTML=c.ico+'&nbsp;'+st.replace(/-/g,' ');
}

/* ── D-PAD ───────────────────────────────────────────── */
var _dt=null;
function hold(c){
  motor(c);clearInterval(_dt);
  _dt=setInterval(function(){motor(c);},90);
}
function rel(){
  clearInterval(_dt);_dt=null;motor('stop');
}
function motor(c){
  fetch('/api/motor/'+c,{method:'POST'})
    .then(function(r){return r.json();})
    .then(function(d){setStatus(d.motor_state);})
    .catch(function(){});
}

/* ── PROPORTIONAL JOYSTICK ───────────────────────────── */
(function(){
  var cv=document.getElementById('jc'),ctx=cv.getContext('2d');

  // Responsive canvas: use container width on mobile
  function resize(){
    var isMobile=window.innerWidth<=680;
    var target=isMobile?Math.min(window.innerWidth-48,300):240;
    cv.style.width=target+'px';cv.style.height=target+'px';
  }
  resize();
  window.addEventListener('resize',resize);

  var SZ=cv.width,CX=SZ/2,CY=SZ/2;
  var OR=SZ/2-6,KR=36,MAX=OR-KR;
  var kx=CX,ky=CY,tx=CX,ty=CY;
  var nx=0,ny=0,pid=null;
  var SP=0.20,SN=0.5,API=55,la=0;

  cv.addEventListener('pointerdown',function(e){
    e.preventDefault();cv.setPointerCapture(e.pointerId);
    pid=e.pointerId;mv(e);
  });
  cv.addEventListener('pointermove',function(e){
    if(e.pointerId!==pid)return;e.preventDefault();mv(e);
  });
  cv.addEventListener('pointerup',snap);
  cv.addEventListener('pointercancel',snap);

  function snap(e){if(e.pointerId!==pid)return;pid=null;tx=CX;ty=CY;}
  function mv(e){
    var r=cv.getBoundingClientRect();
    var sx=SZ/r.width,sy=SZ/r.height;
    var dx=(e.clientX-r.left)*sx-CX,dy=(e.clientY-r.top)*sy-CY;
    var d=Math.hypot(dx,dy);
    if(d>MAX){dx*=MAX/d;dy*=MAX/d;}
    tx=CX+dx;ty=CY+dy;
  }

  function frame(){
    kx+=(tx-kx)*SP;ky+=(ty-ky)*SP;
    if(!pid&&Math.hypot(kx-CX,ky-CY)<SN){kx=CX;ky=CY;}
    nx=(kx-CX)/MAX;ny=-(ky-CY)/MAX;
    draw();
    var now=Date.now();
    if(now-la>=API){la=now;sendJoy();}
    var vx=document.getElementById('vx'),vy=document.getElementById('vy');
    if(vx)vx.textContent=nx.toFixed(2);
    if(vy)vy.textContent=ny.toFixed(2);
    var hx=document.getElementById('hx'),hy=document.getElementById('hy');
    if(hx)hx.textContent=nx.toFixed(2);
    if(hy)hy.textContent=ny.toFixed(2);
    requestAnimationFrame(frame);
  }

  function sendJoy(){
    fetch('/api/joystick',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({x:nx,y:ny})})
    .then(function(r){return r.json();})
    .then(function(d){setStatus(d.motor_state);})
    .catch(function(){});
  }

  function draw(){
    ctx.clearRect(0,0,SZ,SZ);
    var mag=Math.hypot(nx,ny),on=pid!==null;

    // Minimalist Pure White Base Track
    ctx.shadowColor='rgba(0,0,0,0.06)';
    ctx.shadowBlur=16;
    ctx.beginPath();ctx.arc(CX,CY,OR,0,Math.PI*2);
    ctx.fillStyle='#FFFFFF';ctx.fill();
    ctx.shadowBlur=0;

    // Outer Track Ring Border
    ctx.beginPath();ctx.arc(CX,CY,OR,0,Math.PI*2);
    ctx.strokeStyle=on?'#1C1C1E':'#E5E5EA';
    ctx.lineWidth=on?2.5:1.5;ctx.stroke();

    // Subtle Cardinal Tick Marks
    ctx.strokeStyle='rgba(0,0,0,0.08)';ctx.lineWidth=1;
    for(var i=0;i<8;i++){
      var a=i*Math.PI/4;
      var ix=CX+Math.cos(a)*(OR-10),iy=CY+Math.sin(a)*(OR-10);
      var ox=CX+Math.cos(a)*(OR-3),oy=CY+Math.sin(a)*(OR-3);
      ctx.beginPath();ctx.moveTo(ix,iy);ctx.lineTo(ox,oy);ctx.stroke();
    }

    // Cross-hair Center Guidelines
    ctx.strokeStyle='rgba(0,0,0,0.06)';ctx.lineWidth=1;
    ctx.beginPath();
    ctx.moveTo(CX-OR+10,CY);ctx.lineTo(CX+OR-10,CY);
    ctx.moveTo(CX,CY-OR+10);ctx.lineTo(CX,CY+OR-10);
    ctx.stroke();

    // Vector Motion Line
    if(mag>0.04){
      ctx.beginPath();ctx.moveTo(CX,CY);ctx.lineTo(kx,ky);
      ctx.strokeStyle='#1C1C1E';
      ctx.lineWidth=2.5;ctx.stroke();
    }

    // Floating 3D Metallic Thumbstick Knob
    ctx.shadowColor='rgba(0,0,0,0.18)';
    ctx.shadowBlur=on?14:6;

    var kg=ctx.createRadialGradient(kx-8,ky-8,2,kx,ky,KR);
    kg.addColorStop(0,'#FFFFFF');
    kg.addColorStop(1,'#E5E5EA');

    ctx.beginPath();ctx.arc(kx,ky,KR,0,Math.PI*2);
    ctx.fillStyle=kg;ctx.fill();
    ctx.shadowBlur=0;
    ctx.strokeStyle='#1C1C1E';
    ctx.lineWidth=2;ctx.stroke();

    // Center Pip Indicator
    ctx.beginPath();ctx.arc(kx,ky,5,0,Math.PI*2);
    ctx.fillStyle='#1C1C1E';ctx.fill();
  }

  requestAnimationFrame(frame);
}());

/* ── TELEMETRY SYNC ──────────────────────────────────── */
setInterval(function(){
  fetch('/api/telemetry')
    .then(function(r){return r.json();})
    .then(function(d){
      if(d.servo_angle!=null)setServo(Math.round(d.servo_angle));
      if(d.status)setStatus(d.status);
    }).catch(function(){});
},700);

/* ── CAPTURE ─────────────────────────────────────────── */
var _recOn=false,_recSec=0,_recTim=null;

function capturePhoto(){
  fetch('/api/capture/photo',{method:'POST'})
    .then(function(r){return r.json();})
    .then(function(d){
      if(d.status==='success'){
        setMsg('\u2713 '+d.filename,'ok');
        _flash();loadGallery();
      } else { setMsg('\u2715 Camera not ready'); }
    }).catch(function(){setMsg('\u2715 Failed');});
}

function toggleVideo(){
  if(_recOn)_stopVid();else _startVid();
}

function _startVid(){
  fetch('/api/capture/video/start',{method:'POST'})
    .then(function(r){return r.json();})
    .then(function(d){
      if(d.status==='recording'){
        _recOn=true;_recSec=0;
        setMsg('\u25cf REC: '+d.filename,'ok');
        _setVidUI(true);
        _recTim=setInterval(function(){
          _recSec++;
          var m=Math.floor(_recSec/60),s=_recSec%60;
          var t=(m<10?'0':'')+m+':'+(s<10?'0':'')+s;
          ['rec-t','rec-t2','rec-t-m'].forEach(function(id){
            var el=document.getElementById(id);if(el)el.textContent=t;
          });
        },1000);
      } else { setMsg('\u2715 Cannot start'); }
    }).catch(function(){setMsg('\u2715 Failed');});
}

function _stopVid(){
  fetch('/api/capture/video/stop',{method:'POST'})
    .then(function(r){return r.json();})
    .then(function(d){
      _recOn=false;clearInterval(_recTim);_recTim=null;
      _setVidUI(false);
      if(d.status==='saved'){setMsg('\u2713 Saved: '+d.filename,'ok');setTimeout(loadGallery,1500);}
      else setMsg('Stopped');
    }).catch(function(){setMsg('\u2715 Failed');});
}

function _setVidUI(on){
  var btns=[
    ['btn-vid','vid-ico','vid-lbl'],
    ['btn-vid2','vid-ico2','vid-lbl2'],
    ['btn-vid-m','vid-ico-m','vid-lbl-m'],
  ];
  btns.forEach(function(ids){
    var b=document.getElementById(ids[0]);
    var i=document.getElementById(ids[1]);
    var l=document.getElementById(ids[2]);
    if(b){if(on)b.classList.add('recording');else b.classList.remove('recording');}
    if(i)i.innerHTML=on?'&#9632;':'&#9654;';
    if(l)l.textContent=on?'Stop':'Record';
  });
  ['rec-bar','rec-bar2','rec-bar-m'].forEach(function(id){
    var el=document.getElementById(id);
    if(el){if(on)el.classList.add('on');else el.classList.remove('on');}
  });
}

function setMsg(msg,cls){
  ['cap-msg','cap-msg2','cap-msg-m'].forEach(function(id){
    var el=document.getElementById(id);
    if(!el)return;
    el.textContent=msg;
    el.className='cap-msg'+(cls?' '+cls:'');
  });
}

function _flash(){
  var o=document.createElement('div');
  o.className='flash';document.body.appendChild(o);
  setTimeout(function(){o.remove();},350);
}

function loadGallery(){
  fetch('/api/media/list')
    .then(function(r){return r.json();})
    .then(function(items){
      ['gallery','gallery-m'].forEach(function(gid){
        var g=document.getElementById(gid);
        if(!g)return;
        if(!items.length){g.innerHTML='<div class="g-empty">No captures yet</div>';return;}
        var h='';
        items.forEach(function(it){
          var dl='/api/media/'+it.name;
          h+='<div class="gitem">';
          if(it.type==='photo'){
            h+='<img src="'+dl+'" loading="lazy" alt="">';
          } else {
            h+='<div class="gitem-vid">&#127909;</div>';
          }
          h+='<div class="gitem-t">'+(it.type==='photo'?'JPG':'VID')+'</div>';
          h+='<a class="g-dl" href="'+dl+'" download="'+it.name+'">&#8595;</a>';
          h+='</div>';
        });
        g.innerHTML=h;
      });
    }).catch(function(){});
}

loadGallery();
setInterval(loadGallery,15000);
</script>
</body>
</html>
"""

if __name__ == "__main__":
    server = DashboardServer(config=DashboardConfig())
    server.start()
