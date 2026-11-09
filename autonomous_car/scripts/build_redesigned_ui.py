# -*- coding: utf-8 -*-
"""
Builder script to generate the redesigned Cybernetic Ground Control Station (GCS)
web dashboard for the Autonomous RC Car.
"""
import os
import sys

def build_template():
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    dash_dir = os.path.join(os.path.dirname(curr_dir), "dashboard")
    template_path = os.path.join(dash_dir, "template.html")
    backup_path = os.path.join(dash_dir, "template.html.bak")

    # Read original JS from lines 1274 onwards of backup
    if not os.path.exists(backup_path):
        with open(template_path, "r", encoding="utf-8") as f:
            content = f.read()
    else:
        with open(backup_path, "r", encoding="utf-8") as f:
            content = f.read()

    lines = content.splitlines(True)
    js_start = 1273
    for idx, l in enumerate(lines):
        if "<script>" in l:
            js_start = idx
            break

    orig_js = "".join(lines[js_start+1:])
    # Remove trailing </body></html>
    if "</script>" in orig_js:
        orig_js = orig_js[:orig_js.rfind("</script>")]

    # Modify the joystick canvas draw() function inside the JS to use sleek dark cyberpunk styling
    old_draw_start = "function draw(){\n    ctx.clearRect(0,0,SZ,SZ);\n    var mag=Math.hypot(nx,ny),on=pid!==null;\n\n    // Minimalist Pure White Base Track"
    # Let's check if old_draw_start exists or find it
    custom_draw = """  function draw(){
    ctx.clearRect(0,0,SZ,SZ);
    var mag=Math.hypot(nx,ny),on=pid!==null;

    // Tactical Cybernetic Dark Base Track
    var bgGrad = ctx.createRadialGradient(CX, CY, 10, CX, CY, OR);
    bgGrad.addColorStop(0, '#101625');
    bgGrad.addColorStop(0.75, '#0B0F19');
    bgGrad.addColorStop(1, '#070A11');
    ctx.beginPath(); ctx.arc(CX, CY, OR, 0, Math.PI * 2);
    ctx.fillStyle = bgGrad; ctx.fill();

    // Outer Track Ring Border with Glowing Accent
    ctx.beginPath(); ctx.arc(CX, CY, OR, 0, Math.PI * 2);
    ctx.strokeStyle = on ? 'rgba(0, 240, 255, 0.75)' : 'rgba(255, 255, 255, 0.12)';
    ctx.lineWidth = on ? 2.5 : 1.5;
    ctx.shadowColor = on ? '#00F0FF' : 'transparent';
    ctx.shadowBlur = on ? 12 : 0;
    ctx.stroke();
    ctx.shadowBlur = 0;

    // Concentric Metric Deflection Rings (33% and 66%)
    [0.35, 0.70].forEach(function(rPct){
      ctx.beginPath(); ctx.arc(CX, CY, MAX * rPct, 0, Math.PI * 2);
      ctx.strokeStyle = 'rgba(0, 240, 255, 0.08)';
      ctx.setLineDash([3, 4]); ctx.lineWidth = 1; ctx.stroke();
      ctx.setLineDash([]);
    });

    // Cardinal Tick Marks in Luminous Cyan
    ctx.strokeStyle = on ? 'rgba(0, 240, 255, 0.5)' : 'rgba(255, 255, 255, 0.2)';
    ctx.lineWidth = 1.5;
    for(var i=0; i<8; i++){
      var a = i * Math.PI / 4;
      var ix = CX + Math.cos(a) * (OR - 10), iy = CY + Math.sin(a) * (OR - 10);
      var ox = CX + Math.cos(a) * (OR - 3),  oy = CY + Math.sin(a) * (OR - 3);
      ctx.beginPath(); ctx.moveTo(ix, iy); ctx.lineTo(ox, oy); ctx.stroke();
    }

    // Cross-hair Center Guidelines
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)'; ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(CX - OR + 12, CY); ctx.lineTo(CX + OR - 12, CY);
    ctx.moveTo(CX, CY - OR + 12); ctx.lineTo(CX, CY + OR - 12);
    ctx.stroke();

    // Vector Motion Beam from Center to Thumb
    if(mag > 0.04){
      ctx.beginPath(); ctx.moveTo(CX, CY); ctx.lineTo(kx, ky);
      ctx.strokeStyle = on ? '#00F0FF' : 'rgba(0, 240, 255, 0.4)';
      ctx.lineWidth = on ? 3 : 2;
      ctx.shadowColor = '#00F0FF'; ctx.shadowBlur = on ? 8 : 2;
      ctx.stroke();
      ctx.shadowBlur = 0;
    }

    // Floating 3D Cyber Thumbstick Knob
    ctx.shadowColor = on ? 'rgba(0, 240, 255, 0.5)' : 'rgba(0, 0, 0, 0.5)';
    ctx.shadowBlur = on ? 16 : 8;

    var kg = ctx.createRadialGradient(kx - 6, ky - 6, 2, kx, ky, KR);
    kg.addColorStop(0, on ? '#243452' : '#1C2538');
    kg.addColorStop(0.7, '#131B2A');
    kg.addColorStop(1, '#0C121D');

    ctx.beginPath(); ctx.arc(kx, ky, KR, 0, Math.PI * 2);
    ctx.fillStyle = kg; ctx.fill();
    ctx.shadowBlur = 0;

    // Knob Rim Accent
    ctx.beginPath(); ctx.arc(kx, ky, KR, 0, Math.PI * 2);
    ctx.strokeStyle = on ? '#00F0FF' : 'rgba(255, 255, 255, 0.25)';
    ctx.lineWidth = on ? 2.5 : 1.5; ctx.stroke();

    // Center Pip Indicator
    ctx.beginPath(); ctx.arc(kx, ky, 6, 0, Math.PI * 2);
    ctx.fillStyle = on ? '#00F0FF' : 'rgba(255, 255, 255, 0.6)';
    ctx.shadowColor = on ? '#00F0FF' : 'transparent';
    ctx.shadowBlur = on ? 10 : 0;
    ctx.fill();
    ctx.shadowBlur = 0;
  }"""

    # Replace draw() function in JS
    draw_marker = "  function draw(){\n"
    draw_end_marker = "  /* ── KEYBOARD WASD & ARROW CONTROLS ───────────────── */"
    if draw_marker in orig_js and draw_end_marker in orig_js:
        p1 = orig_js.find(draw_marker)
        p2 = orig_js.find(draw_end_marker)
        orig_js = orig_js[:p1] + custom_draw + "\n\n" + orig_js[p2:]

    # Add Tab View Switcher logic to JS
    tab_js = """
/* ── WORKSPACE VIEW TABS FILTER ──────────────────────── */
function switchViewTab(tabName) {
  var validTabs = ['all', 'vision', 'cockpit', 'autonomy', 'hardware', 'media'];
  if (validTabs.indexOf(tabName) === -1) tabName = 'all';

  try {
    localStorage.setItem('rc_active_workspace_tab', tabName);
  } catch(e) {}

  document.querySelectorAll('.tab-btn').forEach(function(btn){
    btn.classList.toggle('active', btn.getAttribute('data-tab') === tabName);
  });

  var cards = {
    'cam': document.querySelector('.a-cam'),
    'map': document.querySelector('.a-map'),
    'stat': document.querySelector('.a-stat'),
    'servo': document.querySelector('.a-servo'),
    'auto': document.querySelector('.a-auto'),
    'ctrl': document.querySelector('.a-ctrl'),
    'mobile': document.querySelector('.mobile-dock')
  };

  // Reset visibility
  Object.keys(cards).forEach(function(k){
    if (cards[k]) cards[k].classList.remove('tab-hidden');
  });

  if (tabName === 'all') {
    // Show all
  } else if (tabName === 'vision') {
    if (cards.stat) cards.stat.classList.add('tab-hidden');
    if (cards.servo) cards.servo.classList.add('tab-hidden');
    if (cards.auto) cards.auto.classList.add('tab-hidden');
    if (cards.ctrl) cards.ctrl.classList.add('tab-hidden');
    if (cards.mobile) cards.mobile.classList.add('tab-hidden');
  } else if (tabName === 'cockpit') {
    if (cards.map) cards.map.classList.add('tab-hidden');
    if (cards.auto) cards.auto.classList.add('tab-hidden');
  } else if (tabName === 'autonomy') {
    if (cards.stat) cards.stat.classList.add('tab-hidden');
    if (cards.servo) cards.servo.classList.add('tab-hidden');
    if (cards.ctrl) cards.ctrl.classList.add('tab-hidden');
  } else if (tabName === 'hardware') {
    if (cards.cam) cards.cam.classList.add('tab-hidden');
    if (cards.map) cards.map.classList.add('tab-hidden');
    if (cards.auto) cards.auto.classList.add('tab-hidden');
  } else if (tabName === 'media') {
    if (cards.map) cards.map.classList.add('tab-hidden');
    if (cards.stat) cards.stat.classList.add('tab-hidden');
    if (cards.servo) cards.servo.classList.add('tab-hidden');
    if (cards.auto) cards.auto.classList.add('tab-hidden');
  }

  // Trigger resize on canvases so they match new layout widths
  setTimeout(function(){
    if (typeof resizeSpatialCanvas === 'function') resizeSpatialCanvas();
  }, 100);
}

// Restore saved workspace tab
try {
  var _savedTab = localStorage.getItem('rc_active_workspace_tab') || 'all';
  switchViewTab(_savedTab);
} catch(e) {}

// Master Emergency Stop Function
function masterEmergencyStop() {
  rel();
  fetch('/api/safety/emergency_stop', {method: 'POST'}).catch(function(){});
  setMsg('⚠️ EMERGENCY STOP ENGAGED — ALL ACTUATORS HALTED', 'error');
  var b = document.getElementById('btn-master-estop');
  if (b) {
    b.classList.add('flashing');
    setTimeout(function(){ b.classList.remove('flashing'); }, 1200);
  }
}
"""
    orig_js += "\n" + tab_js

    # Now let's assemble the new redesigned template
    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>RC Car • Tactical Ground Control Station</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700;800&family=JetBrains+Mono:wght@500;600;700;800&display=swap" rel="stylesheet">
<style>
/* ─────────────────────────────────────────────────────────────
   RESET & FOUNDATION
───────────────────────────────────────────────────────────── */
*, *::before, *::after {{
  box-sizing: border-box;
  margin: 0;
  padding: 0;
  user-select: none;
  -webkit-user-select: none;
  -webkit-tap-highlight-color: transparent;
}}

:root {{
  --bg-deep        : #07090E;
  --bg-base        : #0B0F17;
  --bg-surface     : #101622;
  --bg-card        : rgba(16, 23, 36, 0.88);
  --bg-card-hover  : rgba(22, 32, 50, 0.95);
  --bg-input       : #131B2B;
  --border         : rgba(255, 255, 255, 0.09);
  --border-subtle  : rgba(255, 255, 255, 0.05);
  --border-focus   : rgba(0, 240, 255, 0.45);
  --text-primary   : #F0F4FC;
  --text-secondary : #8A98B2;
  --text-muted     : #556480;
  --accent-cyan    : #00F0FF;
  --accent-green   : #00E599;
  --accent-amber   : #FFB020;
  --accent-red     : #FF3B30;
  --accent-purple  : #A855F7;
  --r-sm           : 8px;
  --r-md           : 14px;
  --r-lg           : 20px;
  --font-sans      : 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
  --font-mono      : 'JetBrains Mono', 'SF Mono', Menlo, Consolas, monospace;
}}

html, body {{
  height: 100%;
  overscroll-behavior: none;
}}

body {{
  background-color: var(--bg-deep);
  color: var(--text-primary);
  font-family: var(--font-sans);
  min-height: 100vh;
  touch-action: pan-y;
  overflow-x: hidden;
  /* Tactical radar grid background */
  background-image: 
    radial-gradient(circle at 50% 0%, rgba(0, 240, 255, 0.05) 0%, transparent 60%),
    linear-gradient(rgba(255, 255, 255, 0.02) 1px, transparent 1px),
    linear-gradient(90deg, rgba(255, 255, 255, 0.02) 1px, transparent 1px);
  background-size: 100% 100%, 36px 36px, 36px 36px;
  background-attachment: fixed;
}}

/* ─────────────────────────────────────────────────────────────
   TACTICAL FROSTED CARDS
───────────────────────────────────────────────────────────── */
.card {{
  background: var(--bg-card);
  backdrop-filter: blur(18px);
  -webkit-backdrop-filter: blur(18px);
  border-radius: var(--r-md);
  border: 1px solid var(--border);
  box-shadow: 0 10px 32px rgba(0, 0, 0, 0.45), inset 0 1px 0 rgba(255, 255, 255, 0.08);
  position: relative;
  overflow: hidden;
  transition: border-color 0.2s, box-shadow 0.2s;
}}

.card:hover {{
  border-color: rgba(255, 255, 255, 0.16);
}}

/* Illuminated cyber corner brackets */
.card::after {{
  content: '';
  position: absolute;
  top: 0; left: 0; right: 0; height: 1px;
  background: linear-gradient(90deg, transparent, rgba(0, 240, 255, 0.35) 30%, rgba(0, 240, 255, 0.35) 70%, transparent);
  pointer-events: none;
}}

.tab-hidden {{
  display: none !important;
}}

/* Section label */
.lbl {{
  font-size: 11px;
  font-weight: 700;
  letter-spacing: 0.12em;
  text-transform: uppercase;
  color: var(--text-secondary);
  display: flex;
  align-items: center;
  gap: 8px;
}}
.lbl-dot {{
  width: 6px;
  height: 6px;
  border-radius: 50%;
  background: var(--accent-cyan);
  box-shadow: 0 0 8px var(--accent-cyan);
  flex-shrink: 0;
}}

/* ─────────────────────────────────────────────────────────────
   TOPBAR & STATUS COMMAND DECK
───────────────────────────────────────────────────────────── */
.topbar {{
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 16px 20px 12px;
  max-width: 1400px;
  margin: 0 auto;
  gap: 14px;
  flex-wrap: wrap;
}}
.topbar-left {{
  display: flex;
  align-items: center;
  gap: 12px;
}}
.app-icon {{
  width: 38px;
  height: 38px;
  border-radius: 10px;
  background: linear-gradient(135deg, #16243C, #0C1322);
  border: 1px solid rgba(0, 240, 255, 0.3);
  color: var(--accent-cyan);
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
  box-shadow: 0 0 14px rgba(0, 240, 255, 0.2);
}}
.app-name {{
  font-size: 1.12rem;
  font-weight: 800;
  letter-spacing: -0.01em;
  color: #FFFFFF;
  display: flex;
  align-items: center;
  gap: 6px;
}}
.app-badge-callsign {{
  font-size: 9px;
  font-family: var(--font-mono);
  font-weight: 700;
  color: var(--accent-cyan);
  background: rgba(0, 240, 255, 0.12);
  border: 1px solid rgba(0, 240, 255, 0.3);
  padding: 2px 6px;
  border-radius: 4px;
}}
.app-sub {{
  font-size: 10.5px;
  color: var(--text-secondary);
  margin-top: 2px;
  font-weight: 500;
  letter-spacing: 0.02em;
}}

/* Topbar Pills & Action Bar */
.topbar-right {{
  display: flex;
  align-items: center;
  gap: 8px;
  flex-wrap: wrap;
}}

.live-pill {{
  display: flex;
  align-items: center;
  gap: 6px;
  background: rgba(255, 255, 255, 0.05);
  border: 1px solid var(--border);
  border-radius: 20px;
  padding: 5px 12px;
  font-size: 10.5px;
  font-weight: 700;
  letter-spacing: 0.05em;
  color: var(--text-primary);
  font-family: var(--font-mono);
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.2);
  transition: all 0.2s;
}}
.live-dot {{
  width: 7px;
  height: 7px;
  border-radius: 50%;
  background: var(--accent-green);
  box-shadow: 0 0 8px var(--accent-green);
  animation: blink 2s infinite;
}}
@keyframes blink {{
  0%, 100% {{ opacity: 1; }}
  50% {{ opacity: 0.3; }}
}}

/* Master E-STOP in Header */
.top-estop-btn {{
  background: linear-gradient(135deg, #FF3B30, #C21A10);
  border: 1px solid rgba(255, 59, 48, 0.6);
  border-radius: 20px;
  color: #FFFFFF;
  padding: 6px 14px;
  font-size: 10.5px;
  font-weight: 800;
  letter-spacing: 0.06em;
  cursor: pointer;
  display: flex;
  align-items: center;
  gap: 6px;
  box-shadow: 0 0 16px rgba(255, 59, 48, 0.45);
  transition: transform 0.1s, box-shadow 0.2s;
}}
.top-estop-btn:hover {{
  box-shadow: 0 0 24px rgba(255, 59, 48, 0.75);
}}
.top-estop-btn:active, .top-estop-btn.flashing {{
  transform: scale(0.95);
  background: #FFFFFF;
  color: #FF3B30;
}}

/* ─────────────────────────────────────────────────────────────
   WORKSPACE FILTER TABS (NAVIGATION BAR)
───────────────────────────────────────────────────────────── */
.tabs-nav {{
  max-width: 1400px;
  margin: 0 auto 16px;
  padding: 0 20px;
  display: flex;
  align-items: center;
  gap: 8px;
  overflow-x: auto;
  scrollbar-width: none;
}}
.tabs-nav::-webkit-scrollbar {{ display: none; }}

.tab-btn {{
  background: var(--bg-surface);
  border: 1px solid var(--border);
  border-radius: var(--r-sm);
  padding: 7px 14px;
  font-size: 11px;
  font-weight: 700;
  letter-spacing: 0.04em;
  color: var(--text-secondary);
  cursor: pointer;
  white-space: nowrap;
  display: inline-flex;
  align-items: center;
  gap: 6px;
  transition: all 0.15s ease-out;
}}
.tab-btn:hover {{
  background: var(--bg-card-hover);
  color: var(--text-primary);
  border-color: rgba(255, 255, 255, 0.18);
}}
.tab-btn.active {{
  background: rgba(0, 240, 255, 0.12);
  color: var(--accent-cyan);
  border-color: rgba(0, 240, 255, 0.45);
  box-shadow: 0 0 12px rgba(0, 240, 255, 0.15);
}}

/* ─────────────────────────────────────────────────────────────
   MAIN DESKTOP GRID LAYOUT
───────────────────────────────────────────────────────────── */
.layout {{
  display: grid;
  grid-template-columns: 1.15fr 1fr;
  gap: 18px;
  padding: 0 20px 32px;
  max-width: 1400px;
  margin: 0 auto;
  align-items: stretch;
}}
.a-cam   {{ grid-column: 1; grid-row: 1; padding: 18px; display: flex; flex-direction: column; min-height: 0; }}
.a-map   {{ grid-column: 2; grid-row: 1; padding: 18px; display: flex; flex-direction: column; min-height: 0; }}
.a-stat  {{ grid-column: 1; grid-row: 2; padding: 18px; }}
.a-servo {{ grid-column: 2; grid-row: 2; padding: 18px; }}
.a-auto  {{ grid-column: 1 / -1; grid-row: 3; padding: 18px; }}
.a-ctrl  {{ grid-column: 1 / -1; grid-row: 4; padding: 18px; }}

@media(max-width: 900px) {{
  .layout {{
    grid-template-columns: 1fr;
  }}
  .a-cam, .a-map, .a-stat, .a-servo, .a-auto, .a-ctrl {{
    grid-column: 1;
  }}
}}

@media(max-width: 680px) {{
  .layout {{
    display: flex;
    flex-direction: column;
    padding: 0 14px 28px;
    gap: 14px;
  }}
  .a-cam       {{ order: 1; padding: 14px; }}
  .a-map       {{ order: 2; padding: 14px; }}
  .a-auto      {{ order: 3; padding: 14px; }}
  .a-ctrl      {{ order: 4; }}
  .a-stat      {{ display: none !important; }}
  .a-servo     {{ display: none !important; }}
  .mobile-dock {{ order: 5; display: flex; }}
  .desktop-only, .dpad-section, .gallery-wrap {{
    display: none !important;
  }}
}}

/* ─────────────────────────────────────────────────────────────
   CAMERA & LIVE FIELD ALIGNMENT
───────────────────────────────────────────────────────────── */
.cam-header {{
  display: flex;
  align-items: center;
  justify-content: space-between;
  margin-bottom: 14px;
  flex-wrap: wrap;
  gap: 10px;
}}
.cam-tools {{
  display: flex;
  align-items: center;
  gap: 6px;
  flex-wrap: wrap;
}}
.tool-btn {{
  background: var(--bg-surface);
  border: 1px solid var(--border);
  border-radius: var(--r-sm);
  padding: 6px 11px;
  font-size: 11px;
  font-weight: 600;
  color: var(--text-primary);
  cursor: pointer;
  display: inline-flex;
  align-items: center;
  gap: 5px;
  transition: all 0.14s;
  font-family: var(--font-sans);
  box-shadow: 0 2px 6px rgba(0, 0, 0, 0.2);
}}
.tool-btn:hover {{
  background: var(--bg-card-hover);
  border-color: rgba(255, 255, 255, 0.2);
}}
.tool-btn.active {{
  background: rgba(0, 240, 255, 0.15);
  color: var(--accent-cyan);
  border-color: rgba(0, 240, 255, 0.45);
  box-shadow: 0 0 10px rgba(0, 240, 255, 0.2);
}}

.cam-frame {{
  border-radius: var(--r-md);
  overflow: hidden;
  background: #04060A;
  border: 1px solid rgba(255, 255, 255, 0.08);
  position: relative;
  flex: 1;
  min-height: 380px;
  display: flex;
  align-items: center;
  justify-content: center;
  box-shadow: inset 0 0 30px rgba(0, 0, 0, 0.8);
}}
.cam-frame img {{
  width: 100%;
  height: 100%;
  max-width: 100%;
  max-height: 100%;
  object-fit: contain;
  display: block;
  transition: transform 0.18s ease-out;
}}
.cam-frame img.cover-mode {{
  object-fit: cover;
}}

/* Alignment HUD Overlay */
.cam-align-overlay {{
  position: absolute;
  inset: 0;
  pointer-events: none;
}}
.grid-svg {{
  width: 100%;
  height: 100%;
  display: block;
}}
.grid-badge {{
  position: absolute;
  top: 12px;
  left: 50%;
  transform: translateX(-50%);
  background: rgba(11, 15, 23, 0.9);
  backdrop-filter: blur(10px);
  border: 1px solid rgba(0, 240, 255, 0.35);
  color: var(--accent-cyan);
  padding: 4px 12px;
  border-radius: 14px;
  font-size: 9.5px;
  font-weight: 700;
  letter-spacing: 0.08em;
  display: flex;
  align-items: center;
  gap: 6px;
  box-shadow: 0 4px 14px rgba(0, 0, 0, 0.5);
  white-space: nowrap;
}}

.cam-hud {{
  position: absolute;
  bottom: 0;
  left: 0;
  right: 0;
  background: linear-gradient(transparent, rgba(7, 9, 14, 0.88));
  display: flex;
  align-items: flex-end;
  justify-content: space-between;
  padding: 24px 14px 12px;
  pointer-events: none;
}}
.cam-badge {{
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 10.5px;
  font-weight: 700;
  letter-spacing: 0.07em;
  color: rgba(255, 255, 255, 0.95);
  font-family: var(--font-mono);
  background: rgba(16, 23, 36, 0.75);
  border: 1px solid rgba(255, 255, 255, 0.1);
  padding: 3px 8px;
  border-radius: 6px;
}}
.rec-dot {{
  width: 7px;
  height: 7px;
  border-radius: 50%;
  background: var(--accent-red);
  box-shadow: 0 0 8px var(--accent-red);
  animation: blink 1s infinite;
}}
.cam-res {{
  font-size: 10px;
  color: var(--text-secondary);
  font-weight: 600;
  font-family: var(--font-mono);
}}

/* Tap to Navigate Pin Reticle */
.target-pin-layer {{
  position: absolute;
  inset: 0;
  pointer-events: none;
  z-index: 10;
}}
.pin-reticle {{
  position: absolute;
  transform: translate(-50%, -50%);
  width: 46px;
  height: 46px;
  pointer-events: none;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: all 0.15s cubic-bezier(0.16, 1, 0.3, 1);
}}
.pin-radar {{
  position: absolute;
  width: 100%;
  height: 100%;
  border-radius: 50%;
  border: 2px solid var(--accent-cyan);
  background: rgba(0, 240, 255, 0.2);
  animation: pin-pulse 1.8s cubic-bezier(0.25, 1, 0.5, 1) infinite;
}}
@keyframes pin-pulse {{
  0% {{ transform: scale(0.6); opacity: 1; }}
  100% {{ transform: scale(1.8); opacity: 0; }}
}}
.pin-center {{
  width: 11px;
  height: 11px;
  border-radius: 50%;
  background: var(--accent-cyan);
  border: 2px solid #FFFFFF;
  box-shadow: 0 0 12px var(--accent-cyan);
}}
.pin-tag {{
  position: absolute;
  bottom: -26px;
  left: 50%;
  transform: translateX(-50%);
  background: rgba(11, 15, 23, 0.94);
  backdrop-filter: blur(8px);
  color: #FFFFFF;
  border: 1px solid rgba(0, 240, 255, 0.4);
  padding: 3px 9px;
  border-radius: 8px;
  font-size: 9.5px;
  font-weight: 700;
  white-space: nowrap;
  letter-spacing: 0.04em;
  font-family: var(--font-mono);
  box-shadow: 0 4px 14px rgba(0, 0, 0, 0.5);
}}
.pin-clear-btn {{
  position: absolute;
  top: 12px;
  right: 12px;
  background: rgba(255, 59, 48, 0.9);
  backdrop-filter: blur(10px);
  color: #FFFFFF;
  border: 1px solid rgba(255, 255, 255, 0.2);
  padding: 5px 12px;
  border-radius: 20px;
  font-size: 10.5px;
  font-weight: 700;
  letter-spacing: 0.05em;
  cursor: pointer;
  display: flex;
  align-items: center;
  gap: 5px;
  box-shadow: 0 2px 10px rgba(255, 59, 48, 0.45);
  z-index: 20;
  transition: transform 0.12s, background 0.12s;
}}
.pin-clear-btn:active {{
  transform: scale(0.94);
}}

/* ─────────────────────────────────────────────────────────────
   3D SPATIAL WORLD MAP CARD & ACTIVE VISION FRUSTUM
───────────────────────────────────────────────────────────── */
.map-header {{
  display: flex;
  align-items: center;
  justify-content: space-between;
  margin-bottom: 14px;
  flex-wrap: wrap;
  gap: 10px;
}}
.map-tools {{
  display: flex;
  align-items: center;
  gap: 6px;
  flex-wrap: wrap;
}}
.map-badge {{
  font-size: 10.5px;
  font-weight: 700;
  letter-spacing: 0.04em;
  padding: 5px 10px;
  border-radius: var(--r-sm);
  background: rgba(0, 240, 255, 0.12);
  color: var(--accent-cyan);
  border: 1px solid rgba(0, 240, 255, 0.3);
  font-family: var(--font-mono);
  display: inline-flex;
  align-items: center;
  gap: 5px;
}}
.spatial-canvas-wrap {{
  position: relative;
  width: 100%;
  flex: 1;
  background: #090D15;
  border-radius: var(--r-md);
  overflow: hidden;
  border: 1px solid rgba(255, 255, 255, 0.08);
  min-height: 380px;
  cursor: crosshair;
  box-shadow: inset 0 2px 14px rgba(0, 0, 0, 0.6);
  display: flex;
  align-items: center;
  justify-content: center;
}}
#spatial-map-canvas {{
  width: 100%;
  height: 100%;
  display: block;
}}
.map-hud-overlay {{
  position: absolute;
  top: 12px;
  left: 12px;
  display: flex;
  flex-direction: column;
  gap: 6px;
  pointer-events: none;
}}
.map-hud-pill {{
  background: rgba(11, 15, 23, 0.88);
  backdrop-filter: blur(10px);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 8px;
  padding: 4px 10px;
  display: flex;
  gap: 7px;
  align-items: center;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.4);
}}
.map-hud-k {{
  font-size: 9px;
  font-weight: 700;
  color: var(--text-muted);
  letter-spacing: 0.08em;
  font-family: var(--font-mono);
}}
.map-hud-v {{
  font-size: 10.5px;
  font-weight: 700;
  color: #FFFFFF;
  font-family: var(--font-mono);
}}
.map-hint-pill {{
  position: absolute;
  bottom: 12px;
  left: 50%;
  transform: translateX(-50%);
  background: rgba(11, 15, 23, 0.9);
  backdrop-filter: blur(10px);
  border: 1px solid rgba(0, 240, 255, 0.25);
  border-radius: 20px;
  padding: 5px 14px;
  font-size: 9.5px;
  font-weight: 700;
  color: var(--accent-cyan);
  letter-spacing: 0.06em;
  display: flex;
  align-items: center;
  gap: 6px;
  pointer-events: none;
  white-space: nowrap;
  box-shadow: 0 4px 14px rgba(0, 0, 0, 0.5);
  font-family: var(--font-mono);
}}

/* ─────────────────────────────────────────────────────────────
   STATUS CARD (DESKTOP)
───────────────────────────────────────────────────────────── */
.st-wrap {{
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 12px;
}}
.st-ring {{
  position: relative;
  width: 90px;
  height: 90px;
}}
.st-ring svg {{
  width: 90px;
  height: 90px;
  transform: rotate(-90deg);
}}
.ring-bg {{
  fill: none;
  stroke: rgba(255, 255, 255, 0.06);
  stroke-width: 6;
}}
.ring-arc {{
  fill: none;
  stroke: var(--accent-cyan);
  stroke-width: 6;
  stroke-linecap: round;
  stroke-dasharray: 215;
  stroke-dashoffset: 60;
  filter: drop-shadow(0 0 6px var(--accent-cyan));
  transition: stroke 0.35s, stroke-dashoffset 0.4s;
}}
.st-ico {{
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  font-size: 1.4rem;
  color: #FFFFFF;
}}
.st-label {{
  font-size: 1.15rem;
  font-weight: 800;
  letter-spacing: 0.02em;
  color: #FFFFFF;
  font-family: var(--font-mono);
}}
.st-sub {{
  font-size: 10px;
  color: var(--text-secondary);
  margin-top: 2px;
  text-transform: uppercase;
  letter-spacing: 0.08em;
}}
.hud-row {{
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 8px;
  width: 100%;
}}
.hud-box {{
  background: var(--bg-surface);
  border: 1px solid var(--border);
  border-radius: var(--r-sm);
  padding: 8px 10px;
}}
.hud-k {{
  font-size: 9.5px;
  text-transform: uppercase;
  letter-spacing: 0.08em;
  color: var(--text-muted);
  font-weight: 700;
  margin-bottom: 2px;
  font-family: var(--font-mono);
}}
.hud-v {{
  font-size: 0.92rem;
  font-weight: 800;
  font-variant-numeric: tabular-nums;
  color: #FFFFFF;
  font-family: var(--font-mono);
}}

/* ─────────────────────────────────────────────────────────────
   SERVO CARD (DESKTOP)
───────────────────────────────────────────────────────────── */
.sv-top {{
  display: flex;
  align-items: center;
  gap: 14px;
  margin-bottom: 12px;
}}
.sv-num {{
  font-size: 2.4rem;
  font-weight: 800;
  letter-spacing: -0.04em;
  line-height: 1;
  color: #FFFFFF;
  font-family: var(--font-mono);
}}
.sv-num sup {{
  font-size: 0.95rem;
  font-weight: 600;
  color: var(--accent-cyan);
}}
.sv-lbl {{
  font-size: 11px;
  color: var(--text-secondary);
  font-weight: 600;
  margin-top: 4px;
}}
.sv-lbl b {{
  color: var(--accent-cyan);
  font-family: var(--font-mono);
}}

/* Sleek Cyber Range Slider */
input[type=range] {{
  -webkit-appearance: none;
  appearance: none;
  width: 100%;
  height: 6px;
  border-radius: 3px;
  background: linear-gradient(to right, var(--accent-cyan) var(--pct, 50%), rgba(255, 255, 255, 0.1) var(--pct, 50%));
  outline: none;
  cursor: pointer;
  margin-bottom: 12px;
}}
input[type=range]::-webkit-slider-thumb {{
  -webkit-appearance: none;
  width: 20px;
  height: 20px;
  border-radius: 50%;
  background: #FFFFFF;
  border: 3px solid var(--accent-cyan);
  box-shadow: 0 0 10px var(--accent-cyan);
  cursor: grab;
  transition: transform 0.1s;
}}
input[type=range]::-webkit-slider-thumb:active {{
  transform: scale(1.2);
}}
input[type=range]::-moz-range-thumb {{
  width: 20px;
  height: 20px;
  border-radius: 50%;
  background: #FFFFFF;
  border: 3px solid var(--accent-cyan);
  cursor: grab;
}}

.sv-btns {{
  display: flex;
  gap: 6px;
}}
.sv-btn {{
  flex: 1;
  height: 34px;
  font-size: 11px;
  font-weight: 700;
  font-family: var(--font-sans);
  background: var(--bg-surface);
  color: var(--text-primary);
  border: 1px solid var(--border);
  border-radius: var(--r-sm);
  cursor: pointer;
  transition: all 0.14s;
}}
.sv-btn:hover {{
  background: var(--bg-card-hover);
  border-color: rgba(255, 255, 255, 0.2);
}}
.sv-btn:active {{
  background: rgba(0, 240, 255, 0.2);
}}
.sv-btn-sec {{
  background: var(--bg-surface);
  color: var(--text-secondary);
}}

/* ─────────────────────────────────────────────────────────────
   AUTONOMOUS DRIVING & AI PERCEPTION CARD
───────────────────────────────────────────────────────────── */
.a-auto {{
  display: flex;
  flex-direction: column;
  gap: 14px;
}}
.auto-header {{
  display: flex;
  align-items: center;
  justify-content: space-between;
  flex-wrap: wrap;
  gap: 12px;
}}
.auto-header-badges {{
  display: flex;
  align-items: center;
  gap: 8px;
  flex-wrap: wrap;
}}
.mode-pill, .server-pill {{
  background: var(--bg-surface);
  border: 1px solid var(--border);
  border-radius: 20px;
  padding: 5px 12px;
  font-size: 10.5px;
  font-weight: 700;
  color: var(--text-secondary);
  letter-spacing: 0.05em;
  font-family: var(--font-mono);
  display: inline-flex;
  align-items: center;
  gap: 6px;
  cursor: pointer;
  transition: all 0.15s;
}}
.mode-pill.active {{
  background: rgba(0, 240, 255, 0.15);
  color: var(--accent-cyan);
  border-color: rgba(0, 240, 255, 0.45);
  box-shadow: 0 0 10px rgba(0, 240, 255, 0.15);
}}
.server-pill.connected {{
  background: rgba(0, 229, 153, 0.15);
  border-color: rgba(0, 229, 153, 0.4);
  color: var(--accent-green);
}}

.auto-toggle-btn {{
  background: linear-gradient(135deg, #182236, #0E1624);
  color: #FFFFFF;
  border: 1px solid rgba(255, 255, 255, 0.14);
  border-radius: 20px;
  padding: 7px 16px;
  font-size: 11.5px;
  font-weight: 800;
  font-family: var(--font-sans);
  cursor: pointer;
  display: inline-flex;
  align-items: center;
  gap: 8px;
  box-shadow: 0 2px 10px rgba(0, 0, 0, 0.3);
  transition: all 0.15s;
}}
.auto-toggle-btn.engaged {{
  background: linear-gradient(135deg, #00E599, #00A86B);
  color: #07090E;
  border-color: #00E599;
  box-shadow: 0 0 20px rgba(0, 229, 153, 0.5);
}}
.toggle-switch-dot {{
  width: 9px;
  height: 9px;
  border-radius: 50%;
  background: var(--text-muted);
  transition: background 0.2s;
}}
.auto-toggle-btn.engaged .toggle-switch-dot {{
  background: #07090E;
  box-shadow: 0 0 8px #FFFFFF;
}}

/* Active Action Decision Banner */
.auto-action-bar {{
  background: var(--bg-surface);
  border: 1px solid var(--border);
  border-radius: var(--r-md);
  padding: 14px 16px;
  display: flex;
  align-items: center;
  justify-content: space-between;
  flex-wrap: wrap;
  gap: 12px;
}}
.auto-action-badge {{
  background: #182236;
  color: #FFFFFF;
  font-size: 12px;
  font-weight: 800;
  letter-spacing: 0.08em;
  padding: 6px 14px;
  border-radius: 8px;
  text-transform: uppercase;
  white-space: nowrap;
  font-family: var(--font-mono);
}}
.auto-action-badge.stop    {{ background: var(--accent-red); box-shadow: 0 0 14px rgba(255, 59, 48, 0.5); }}
.auto-action-badge.forward {{ background: var(--accent-green); color: #07090E; box-shadow: 0 0 14px rgba(0, 229, 153, 0.5); }}
.auto-action-badge.avoid   {{ background: var(--accent-cyan); color: #07090E; box-shadow: 0 0 14px rgba(0, 240, 255, 0.5); }}
.auto-action-badge.slow    {{ background: var(--accent-amber); color: #07090E; box-shadow: 0 0 14px rgba(255, 176, 32, 0.5); }}
.auto-action-badge.scan    {{ background: var(--accent-purple); box-shadow: 0 0 14px rgba(168, 85, 247, 0.5); }}

.auto-action-reason {{
  font-size: 12.5px;
  font-weight: 600;
  color: var(--text-primary);
  flex: 1;
  min-width: 220px;
}}
.auto-action-metrics {{
  display: flex;
  gap: 7px;
  font-size: 10.5px;
  font-weight: 700;
  color: var(--text-secondary);
  flex-wrap: wrap;
}}
.auto-badge-pill {{
  background: var(--bg-card);
  border: 1px solid var(--border);
  border-radius: 6px;
  padding: 4px 8px;
  font-family: var(--font-mono);
}}

/* Auto Grid Controls */
.auto-grid-controls {{
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
  gap: 14px;
}}
.auto-ctrl-box {{
  background: var(--bg-surface);
  border: 1px solid var(--border);
  border-radius: var(--r-sm);
  padding: 12px 14px;
  display: flex;
  flex-direction: column;
  justify-content: space-between;
}}
.auto-k {{
  font-size: 10.5px;
  font-weight: 700;
  letter-spacing: 0.08em;
  text-transform: uppercase;
  color: var(--text-secondary);
}}
.mode-btn-group {{
  display: flex;
  gap: 6px;
  margin-top: 8px;
}}
.mode-btn {{
  flex: 1;
  height: 30px;
  border-radius: 6px;
  border: 1px solid var(--border);
  background: var(--bg-card);
  color: var(--text-secondary);
  font-size: 10.5px;
  font-weight: 700;
  font-family: var(--font-sans);
  cursor: pointer;
  transition: all 0.12s;
}}
.mode-btn.active {{
  background: rgba(0, 240, 255, 0.15);
  color: var(--accent-cyan);
  border-color: rgba(0, 240, 255, 0.45);
}}
.clearance-row {{
  display: flex;
  flex-direction: column;
  gap: 6px;
  margin-top: 8px;
}}
.clearance-item {{
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 10.5px;
  color: var(--text-secondary);
  font-weight: 600;
  font-family: var(--font-mono);
}}
.clearance-bar-wrap {{
  flex: 1;
  height: 7px;
  background: rgba(255, 255, 255, 0.08);
  border-radius: 4px;
  overflow: hidden;
}}
.clearance-bar {{
  height: 100%;
  background: var(--accent-green);
  border-radius: 4px;
  transition: width 0.2s, background 0.2s;
}}
.server-input {{
  flex: 1;
  height: 30px;
  border: 1px solid var(--border);
  border-radius: 6px;
  padding: 0 10px;
  font-size: 11px;
  font-family: var(--font-mono);
  background: var(--bg-input);
  color: #FFFFFF;
}}

/* ─────────────────────────────────────────────────────────────
   CONTROLS ROW (TACTILE COCKPIT)
───────────────────────────────────────────────────────────── */
.a-ctrl {{
  display: grid;
  grid-template-columns: 1fr auto 1fr;
  gap: 16px;
  align-items: start;
}}
@media(max-width: 680px) {{
  .a-ctrl {{
    grid-template-columns: 1fr;
    gap: 14px;
  }}
}}

/* D-PAD */
.dpad-section {{
  display: flex;
  flex-direction: column;
  gap: 12px;
}}
.dpad-wrap {{
  display: flex;
  justify-content: center;
}}
.dpad {{
  display: grid;
  grid-template-columns: repeat(3, 58px);
  grid-template-rows: repeat(3, 58px);
  gap: 8px;
}}
.d-btn {{
  border-radius: 14px;
  background: var(--bg-surface);
  border: 1px solid var(--border);
  color: #FFFFFF;
  font-size: 1.15rem;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  cursor: pointer;
  transition: all 0.08s;
  touch-action: manipulation;
  font-family: var(--font-sans);
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
  line-height: 1;
}}
.d-btn:hover {{
  border-color: rgba(0, 240, 255, 0.3);
}}
.d-btn:active, .d-btn.active {{
  background: rgba(0, 240, 255, 0.25);
  color: var(--accent-cyan);
  border-color: var(--accent-cyan);
  transform: scale(0.92);
  box-shadow: 0 0 16px rgba(0, 240, 255, 0.4);
}}
.d-stp {{
  background: rgba(255, 59, 48, 0.15);
  border-color: rgba(255, 59, 48, 0.45);
  color: var(--accent-red);
}}
.d-stp:active, .d-stp.active {{
  background: var(--accent-red);
  color: #FFFFFF;
  transform: scale(0.92);
  box-shadow: 0 0 16px rgba(255, 59, 48, 0.5);
}}
.key-hint {{
  font-size: 8.5px;
  font-weight: 700;
  letter-spacing: 0.05em;
  color: var(--text-muted);
  margin-top: 3px;
  text-transform: uppercase;
  font-family: var(--font-mono);
}}
.d-btn:active .key-hint, .d-btn.active .key-hint {{
  color: var(--accent-cyan);
}}

/* JOYSTICK SECTION */
.joy-section {{
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 12px;
}}
#jc {{
  display: block;
  touch-action: none;
  cursor: grab;
}}
#jc:active {{
  cursor: grabbing;
}}
.joy-meta {{
  display: flex;
  gap: 8px;
  align-items: center;
  flex-wrap: wrap;
  justify-content: center;
}}
.joy-tag {{
  background: var(--bg-surface);
  border: 1px solid var(--border);
  border-radius: 8px;
  padding: 4px 10px;
  font-size: 10.5px;
  color: var(--text-secondary);
  font-weight: 600;
  font-variant-numeric: tabular-nums;
  font-family: var(--font-mono);
}}
.joy-tag b {{
  color: var(--accent-cyan);
}}

/* CAPTURE COLUMN */
.cap-section {{
  display: flex;
  flex-direction: column;
  gap: 10px;
}}

/* Stop Button (Large Red Pill) */
.btn-stop {{
  width: 100%;
  height: 56px;
  border-radius: 100px;
  background: linear-gradient(135deg, #FF3B30, #C21A10);
  border: 1px solid rgba(255, 59, 48, 0.5);
  font-size: 1rem;
  font-weight: 800;
  letter-spacing: 0.05em;
  color: #FFFFFF;
  font-family: var(--font-sans);
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  transition: opacity 0.1s, transform 0.08s;
  box-shadow: 0 0 20px rgba(255, 59, 48, 0.4);
}}
.btn-stop:hover {{
  box-shadow: 0 0 28px rgba(255, 59, 48, 0.65);
}}
.btn-stop:active {{
  transform: scale(0.97);
}}

.cap-row {{
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 8px;
}}
.btn-cap {{
  height: 48px;
  border-radius: var(--r-sm);
  font-size: 0.9rem;
  font-weight: 700;
  letter-spacing: 0.02em;
  font-family: var(--font-sans);
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  transition: all 0.12s;
  border: 1px solid var(--border);
}}
.btn-cap:active {{
  transform: scale(0.95);
}}
.btn-photo {{
  background: var(--bg-surface);
  color: #FFFFFF;
}}
.btn-photo:hover {{
  background: var(--bg-card-hover);
  border-color: rgba(255, 255, 255, 0.2);
}}
.btn-video {{
  background: rgba(255, 59, 48, 0.1);
  border-color: rgba(255, 59, 48, 0.3);
  color: var(--accent-red);
}}
.btn-video.recording {{
  background: rgba(255, 59, 48, 0.25);
  border-color: var(--accent-red);
  box-shadow: 0 0 16px rgba(255, 59, 48, 0.5);
}}

.rec-bar {{
  display: none;
  align-items: center;
  justify-content: center;
  gap: 8px;
  height: 36px;
  border-radius: 8px;
  background: rgba(255, 59, 48, 0.12);
  border: 1px solid rgba(255, 59, 48, 0.3);
  font-size: 12px;
  font-weight: 700;
  color: var(--accent-red);
  letter-spacing: 0.05em;
  font-family: var(--font-mono);
}}
.rec-bar.on {{
  display: flex;
}}
.rec-bar-dot {{
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background: var(--accent-red);
  box-shadow: 0 0 8px var(--accent-red);
  animation: blink 0.7s infinite;
}}
.cap-msg {{
  text-align: center;
  font-size: 11.5px;
  color: var(--text-secondary);
  font-weight: 500;
  min-height: 18px;
  font-family: var(--font-mono);
  transition: color 0.2s;
}}
.cap-msg.ok {{
  color: var(--accent-green);
}}
.cap-msg.error {{
  color: var(--accent-red);
}}

/* ─────────────────────────────────────────────────────────────
   GALLERY
───────────────────────────────────────────────────────────── */
.gallery-wrap {{
  margin-top: 6px;
}}
.gallery-hd {{
  font-size: 10.5px;
  text-transform: uppercase;
  letter-spacing: 0.1em;
  color: var(--text-secondary);
  font-weight: 700;
  margin-bottom: 8px;
}}
.gallery {{
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(84px, 1fr));
  gap: 8px;
}}
.gitem {{
  border-radius: 8px;
  overflow: hidden;
  border: 1px solid var(--border);
  background: #080C14;
  position: relative;
  aspect-ratio: 4/3;
  cursor: pointer;
  transition: transform 0.12s, border-color 0.12s;
}}
.gitem:hover {{
  transform: scale(1.04);
  border-color: var(--accent-cyan);
}}
.gitem img {{
  width: 100%;
  height: 100%;
  object-fit: cover;
  display: block;
}}
.gitem-t {{
  position: absolute;
  top: 4px;
  left: 4px;
  background: rgba(0, 0, 0, 0.75);
  border-radius: 4px;
  font-size: 8.5px;
  font-weight: 800;
  color: #FFFFFF;
  padding: 1px 5px;
  letter-spacing: 0.05em;
  font-family: var(--font-mono);
}}
.gitem-vid {{
  display: flex;
  align-items: center;
  justify-content: center;
  width: 100%;
  height: 100%;
  font-size: 1.6rem;
  color: var(--accent-cyan);
}}
.g-dl {{
  position: absolute;
  inset: 0;
  background: rgba(0, 0, 0, 0.6);
  display: none;
  align-items: center;
  justify-content: center;
  font-size: 1.3rem;
  color: #FFFFFF;
  text-decoration: none;
}}
.gitem:hover .g-dl {{
  display: flex;
}}
.g-empty {{
  grid-column: 1 / -1;
  text-align: center;
  color: var(--text-muted);
  font-size: 11px;
  padding: 16px 0;
}}

/* ─────────────────────────────────────────────────────────────
   MOBILE DOCK
───────────────────────────────────────────────────────────── */
.mobile-dock {{
  display: none;
  flex-direction: column;
  gap: 12px;
  padding: 0 14px 10px;
  max-width: 900px;
  margin: 0 auto;
}}

/* Shutter flash animation */
@keyframes flash {{
  0% {{ opacity: 0.7; }}
  100% {{ opacity: 0; }}
}}
.flash {{
  position: fixed;
  inset: 0;
  background: #FFFFFF;
  pointer-events: none;
  opacity: 0;
  animation: flash 0.3s ease-out forwards;
  z-index: 9999;
}}
</style>
</head>
<body>

<!-- ── TOPBAR STATUS & COMMAND DECK ── -->
<div class="topbar">
  <div class="topbar-left">
    <div class="app-icon">
      <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round">
        <path d="M19 17h2c.6 0 1-.4 1-1v-3c0-.9-.7-1.7-1.5-1.9C18.7 10.6 16 10 16 10s-1.3-1.4-2.2-2.3c-.5-.4-1.1-.7-1.8-.7H5c-.6 0-1.1.4-1.4.9l-1.4 2.9A3.7 3.7 0 0 0 2 12v4c0 .6.4 1 1 1h2"/>
        <circle cx="7" cy="17" r="2"/>
        <path d="M9 17h6"/>
        <circle cx="17" cy="17" r="2"/>
      </svg>
    </div>
    <div>
      <div class="app-name">
        RC CAR GCS
        <span class="app-badge-callsign">MK-3</span>
      </div>
      <div class="app-sub">Raspberry Pi 3 Model B &bull; Dual Wireless (Wi-Fi + BT RFCOMM)</div>
    </div>
  </div>

  <div class="topbar-right">
    <!-- Master Emergency Stop Button -->
    <button class="top-estop-btn" id="btn-master-estop" onclick="masterEmergencyStop()" title="Master Emergency Stop Actuators">
      <svg width="12" height="12" viewBox="0 0 24 24" fill="currentColor"><rect x="4" y="4" width="16" height="16" rx="2"/></svg>
      E-STOP
    </button>

    <!-- Control Mode Pill -->
    <div class="live-pill" id="control-mode-pill" style="color:var(--accent-cyan);border-color:rgba(0,240,255,0.3);background:rgba(0,240,255,0.08);">
      <span id="control-mode-txt">GAMEPAD / MANUAL</span>
    </div>

    <!-- HC-SR04 Ultrasonic Live Sonar Pill -->
    <div class="live-pill" id="sonar-pill" style="color:var(--accent-cyan);border-color:rgba(0,240,255,0.3);" title="HC-SR04 Live Ultrasonic Distance">
      <svg width="11" height="11" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><path d="M2 12a10 10 0 0 1 10-10M2 12a10 10 0 0 0 10 10M6 12a6 6 0 0 1 6-6M6 12a6 6 0 0 0 6 6"/><circle cx="12" cy="12" r="2"/></svg>
      <span id="sonar-txt">SONAR: --</span>
    </div>

    <!-- CPU Thermal Meter -->
    <div class="live-pill" title="Pi 3 CPU Core Temperature">
      <svg width="11" height="11" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2"><path d="M14 14.76V3.5a2.5 2.5 0 0 0-5 0v11.26a4.5 4.5 0 1 0 5 0z"/></svg>
      <span id="cpu-temp">--°C</span>
    </div>

    <!-- Wi-Fi Link Pill -->
    <div class="live-pill" id="wifi-pill" title="Connected via Wi-Fi">
      <div class="live-dot"></div>
      <span id="wifi-txt">WI-FI</span>
    </div>

    <!-- Bluetooth Link Pill -->
    <div class="live-pill" id="bt-pill" style="color:var(--accent-cyan);cursor:pointer;" onclick="showBtInfo()" title="Click for Bluetooth pairing and range details">
      <svg width="10" height="10" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"><path d="m7 7 10 10-5 5V2l5 5L7 17"/></svg>
      <span id="bt-txt">BT READY</span>
    </div>
  </div>
</div>

<!-- ── WORKSPACE VIEW TABS ── -->
<div class="tabs-nav">
  <button class="tab-btn active" data-tab="all" onclick="switchViewTab('all')">
    <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="3" y="3" width="7" height="7"/><rect x="14" y="3" width="7" height="7"/><rect x="14" y="14" width="7" height="7"/><rect x="3" y="14" width="7" height="7"/></svg>
    All Deck
  </button>
  <button class="tab-btn" data-tab="vision" onclick="switchViewTab('vision')">
    <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M23 7l-7 5 7 5V7z"/><rect x="1" y="5" width="15" height="14" rx="2" ry="2"/></svg>
    Vision &amp; Map
  </button>
  <button class="tab-btn" data-tab="cockpit" onclick="switchViewTab('cockpit')">
    <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><path d="M12 8v8M8 12h8"/></svg>
    Pilot Cockpit
  </button>
  <button class="tab-btn" data-tab="autonomy" onclick="switchViewTab('autonomy')">
    <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="3"/><path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1 0 2.83 2 2 0 0 1-2.83 0l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-2 2 2 2 0 0 1-2-2v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83 0 2 2 0 0 1 0-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1-2-2 2 2 0 0 1 2-2h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 0-2.83 2 2 0 0 1 2.83 0l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 2-2 2 2 0 0 1 2 2v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 0 2 2 0 0 1 0 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 2 2 2 2 0 0 1-2 2h-.09a1.65 1.65 0 0 0-1.51 1z"/></svg>
    Autonomy &amp; AI
  </button>
  <button class="tab-btn" data-tab="hardware" onclick="switchViewTab('hardware')">
    <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M14.7 6.3a1 1 0 0 0 0 1.4l1.6 1.6a1 1 0 0 0 1.4 0l3.77-3.77a6 6 0 0 1-7.94 7.94l-6.91 6.91a2.12 2.12 0 0 1-3-3l6.91-6.91a6 6 0 0 1 7.94-7.94l-3.76 3.76z"/></svg>
    Hardware &amp; Tuning
  </button>
  <button class="tab-btn" data-tab="media" onclick="switchViewTab('media')">
    <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="3" y="3" width="18" height="18" rx="2" ry="2"/><circle cx="8.5" cy="8.5" r="1.5"/><polyline points="21 15 16 10 5 21"/></svg>
    Media Studio
  </button>
</div>

<!-- ── MAIN LAYOUT ── -->
<div class="layout">

  <!-- 1. LIVE CAMERA & FIELD ALIGNMENT -->
  <div class="card a-cam">
    <div class="cam-header">
      <div class="lbl"><div class="lbl-dot"></div>Live Video &bull; Field Alignment</div>
      <div class="cam-tools">
        <button class="tool-btn" id="btn-grid" onclick="toggleGrid()" title="Toggle Center Reticle & Heading Guidelines">
          <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><path d="M12 2v20M2 12h20"/></svg>
          Align Guide
        </button>
        <button class="tool-btn" id="btn-fliph" onclick="toggleFlipH()" title="Mirror Horizontal">
          <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M8 3H5a2 2 0 0 0-2 2v14c0 1.1.9 2 2 2h3m8-18h3a2 2 0 0 1 2 2v14a2 2 0 0 1-2 2h-3M12 2v20"/></svg>
          Flip H
        </button>
        <button class="tool-btn" id="btn-flipv" onclick="toggleFlipV()" title="Invert Upside-Down">
          <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M3 8V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2v3M3 16v3a2 2 0 0 0 2 2h14a2 2 0 0 0 2-2v-3M2 12h20"/></svg>
          Flip V
        </button>
        <button class="tool-btn" id="btn-rot" onclick="cycleRotation()" title="Rotate 90°">
          0°
        </button>
        <button class="tool-btn" id="btn-fit" onclick="toggleFitMode()" title="Toggle Fit Uncropped vs Fill">
          Fit
        </button>
        <button class="tool-btn active" id="btn-ai-overlay" onclick="toggleHudOverlay()" title="Toggle AI Bounding Boxes & Corridor HUD">
          <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M2 12s3-7 10-7 10 7 10 7-3 7-10 7-10-7-10-7Z"/><circle cx="12" cy="12" r="3"/></svg>
          AI HUD
        </button>
        <button class="tool-btn" id="btn-quick-auto" onclick="toggleAutoDrive()" title="Toggle Autonomous Driving Mode (Key: Z)">
          <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><polygon points="10 8 16 12 10 16 10 8" fill="currentColor"/></svg>
          Auto Drive
        </button>
      </div>
    </div>

    <div class="cam-frame">
      <img id="cam" src="/video_feed" alt="live camera feed">

      <!-- Interactive Alignment Reticle & Heading Guidelines -->
      <div class="cam-align-overlay" id="cam-grid" style="display:none;">
        <svg class="grid-svg" viewBox="0 0 640 480" preserveAspectRatio="none">
          <rect x="8" y="8" width="624" height="464" rx="8" fill="none" stroke="rgba(0,240,255,0.45)" stroke-width="1.5" stroke-dasharray="8 6"/>
          <text x="20" y="28" fill="rgba(0,240,255,0.9)" font-size="10" font-weight="700" font-family="'JetBrains Mono', monospace">FULL CAMERA FEED SCANNER (640x480)</text>

          <line x1="320" y1="20" x2="320" y2="460" stroke="rgba(255,59,48,0.75)" stroke-width="1.5" stroke-dasharray="6 4"/>
          <text x="326" y="44" fill="rgba(255,59,48,0.9)" font-size="10" font-weight="700" font-family="'JetBrains Mono', monospace">CENTERLINE</text>

          <circle cx="320" cy="240" r="32" fill="none" stroke="rgba(0,240,255,0.85)" stroke-width="1.5"/>
          <circle cx="320" cy="240" r="3" fill="#FF3B30"/>
          <line x1="284" y1="240" x2="356" y2="240" stroke="#00F0FF" stroke-width="2"/>
          <line x1="320" y1="204" x2="320" y2="276" stroke="#00F0FF" stroke-width="2"/>

          <line x1="220" y1="230" x2="220" y2="250" stroke="rgba(255,255,255,0.6)" stroke-width="1.5"/>
          <text x="210" y="224" fill="rgba(255,255,255,0.75)" font-size="9" font-weight="600" font-family="'JetBrains Mono', monospace">-15°</text>
          <line x1="420" y1="230" x2="420" y2="250" stroke="rgba(255,255,255,0.6)" stroke-width="1.5"/>
          <text x="412" y="224" fill="rgba(255,255,255,0.75)" font-size="9" font-weight="600" font-family="'JetBrains Mono', monospace">+15°</text>
        </svg>
        <div class="grid-badge">
          <svg width="8" height="8" viewBox="0 0 24 24" fill="#00E599"><circle cx="12" cy="12" r="10"/></svg>
          FULL CAMERA FEED &bull; COLLISION GUARD ACTIVE
        </div>
      </div>

      <!-- Visual Destination Pin Overlay -->
      <div id="target-pin-layer" class="target-pin-layer" style="display:none;">
        <div class="pin-reticle" id="pin-reticle" style="left:50%;top:50%;">
          <div class="pin-radar"></div>
          <div class="pin-center"></div>
          <div class="pin-tag" id="pin-tag">TARGET: (320, 240)</div>
        </div>
      </div>
      <button id="btn-clear-pin" class="pin-clear-btn" style="display:none;" onclick="clearDestinationPin(event)">
        ✕ Clear Pin
      </button>

      <div class="cam-hud">
        <div class="cam-badge"><div class="rec-dot"></div>LIVE</div>
        <div class="cam-badge" id="cam-sonar-badge" style="background:rgba(0,240,255,0.18);border-color:rgba(0,240,255,0.4);color:var(--accent-cyan);" title="HC-SR04 Ultrasonic Distance">
          <svg width="10" height="10" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" style="display:inline-block;vertical-align:middle;margin-right:2px;"><path d="M2 12a10 10 0 0 1 10-10M2 12a10 10 0 0 0 10 10M6 12a6 6 0 0 1 6-6M6 12a6 6 0 0 0 6 6"/></svg>
          <span id="cam-sonar-txt">HC-SR04: --</span>
        </div>
        <span class="cam-res" id="cam-hud-info">640 &times; 480 &bull; Fit</span>
      </div>
    </div>
  </div>

  <!-- 2. 3D SPATIAL WORLD MAP & ACTIVE VISION FRUSTUM -->
  <div class="card a-map" id="spatial-map-card">
    <div class="map-header">
      <div class="lbl">
        <div class="lbl-dot" id="map-dot"></div>
        3D Spatial World Map &bull; Active Gaze Frustum
      </div>
      <div class="map-tools">
        <span class="map-badge" id="gaze-lock-badge" title="Active camera gaze servo tracking">
          GAZE: LOCKED (90°)
        </span>
        <button class="tool-btn active" id="btn-toggle-gaze" onclick="toggleGazeTracking()" title="Toggle camera pan gaze lock on 3D target">
          Lock Gaze
        </button>
        <button class="tool-btn" onclick="resetCarPose()" title="Reset Dead-Reckoning Odometry Pose to (0,0) 90°">
          Reset (0,0)
        </button>
        <button class="tool-btn" id="btn-clear-3d-target" onclick="clearMapRoute(event)" title="Clear all map route points">
          Clear Route
        </button>
      </div>
    </div>

    <!-- 3D Spatial Canvas Frame -->
    <div class="spatial-canvas-wrap" id="spatial-canvas-wrap">
      <canvas id="spatial-map-canvas" width="600" height="420"></canvas>

      <!-- Interactive Map Overlay HUD -->
      <div class="map-hud-overlay">
        <div class="map-hud-pill">
          <span class="map-hud-k">VEHICLE POSE</span>
          <span class="map-hud-v" id="hud-car-pose">X: 0.0 &bull; Y: 0.0 &bull; 90.0°</span>
        </div>
        <div class="map-hud-pill">
          <span class="map-hud-k">PAN SERVO</span>
          <span class="map-hud-v" id="hud-servo-angle">90.0° (Forward)</span>
        </div>
        <div class="map-hud-pill" id="hud-sonar-pill">
          <span class="map-hud-k">ULTRASONIC</span>
          <span class="map-hud-v" id="hud-sonar-val">-- cm</span>
        </div>
        <div class="map-hud-pill" id="hud-target-pill" style="display:none;">
          <span class="map-hud-k">MAP ROUTE</span>
          <span class="map-hud-v" id="hud-target-info">X: 0 &bull; Y: 0 (0cm)</span>
        </div>
      </div>

      <!-- Quick Guidance Pill -->
      <div class="map-hint-pill">
        <svg width="10" height="10" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><circle cx="12" cy="12" r="10"/><path d="M12 8v8M8 12h8"/></svg>
        CLICK MAP TO ADD ORDERED ROUTE POINTS &bull; SELECT AI TO ENGAGE
      </div>
    </div>
  </div>

  <!-- 3. STATUS (DESKTOP) -->
  <div class="card a-stat desktop-only">
    <div class="lbl" style="margin-bottom:12px;"><div class="lbl-dot"></div>Status &bull; Actuator Telemetry</div>
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
        <div class="hud-box"><div class="hud-k">Servo Pan</div><div class="hud-v"><span id="hs">90</span>°</div></div>
        <div class="hud-box"><div class="hud-k">Signal</div><div class="hud-v" style="color:var(--accent-green)">LIVE</div></div>
      </div>
    </div>
  </div>

  <!-- 4. SERVO & CALIBRATION (DESKTOP) -->
  <div class="card a-servo desktop-only">
    <div class="lbl" style="margin-bottom:12px;"><div class="lbl-dot"></div>Camera Gimbal &bull; GPIO 12 PWM</div>
    <div class="sv-top">
      <svg width="60" height="38" viewBox="0 0 60 38" style="flex-shrink:0">
        <path d="M4 36 A27 27 0 0 1 56 36" fill="none" stroke="rgba(255,255,255,0.08)" stroke-width="4" stroke-linecap="round"/>
        <path id="arc" d="M4 36 A27 27 0 0 1 56 36" fill="none" stroke="#00F0FF" stroke-width="4" stroke-linecap="round"
              stroke-dasharray="85" stroke-dashoffset="42" style="transition:stroke-dashoffset .2s;filter:drop-shadow(0 0 4px #00F0FF);"/>
        <defs><filter id="gf"><feGaussianBlur stdDeviation="2" result="b"/>
          <feMerge><feMergeNode in="b"/><feMergeNode in="SourceGraphic"/></feMerge></filter></defs>
        <circle id="aknob" cx="30" cy="9" r="4.5" fill="#00F0FF" filter="url(#gf)" style="transition:cx .2s,cy .2s"/>
      </svg>
      <div>
        <div class="sv-num"><span id="sdeg">90</span><sup>°</sup></div>
        <div class="sv-lbl">Pan Angle &bull; Trim: <b id="strim">+0.0°</b></div>
      </div>
    </div>

    <input type="range" id="sl" min="25" max="155" value="90"
           oninput="setServo(this.value)" style="--pct:50%">

    <!-- Fine Nudge Controls -->
    <div class="sv-btns">
      <button class="sv-btn sv-btn-sec" onclick="nudgeServo(-5)" title="Nudge 5° Left">-5°</button>
      <button class="sv-btn sv-btn-sec" onclick="nudgeServo(-1)" title="Nudge 1° Left">-1°</button>
      <button class="sv-btn" onclick="setServo(90)">Center</button>
      <button class="sv-btn sv-btn-sec" onclick="nudgeServo(1)" title="Nudge 1° Right">+1°</button>
      <button class="sv-btn sv-btn-sec" onclick="nudgeServo(5)" title="Nudge 5° Right">+5°</button>
    </div>

    <!-- Alignment Calibration Actions -->
    <div style="display:flex;gap:6px;margin-top:10px;">
      <button class="sv-btn sv-btn-sec" style="font-size:10px;height:30px;" onclick="calibrateCurrentAsCenter()" title="Calibrate current position as vehicle true forward center">
        Set Center
      </button>
      <button class="sv-btn sv-btn-sec" style="font-size:10px;height:30px;color:var(--text-muted);" onclick="resetServoTrim()" title="Reset calibration offset to 0°">
        Reset Trim
      </button>
      <button class="sv-btn sv-btn-sec" style="font-size:10px;height:30px;color:var(--accent-green);font-weight:700;" onclick="relaxServo()" title="Stop PWM to cool down servo coils (de-energize)">
        Sleep / Relax
      </button>
    </div>
  </div>

  <!-- 5. AUTONOMOUS DRIVING & AI PERCEPTION CARD -->
  <div class="card a-auto" id="auto-card">
    <div class="auto-header">
      <div class="lbl">
        <div class="lbl-dot" id="auto-dot"></div>
        Autonomous Driving &bull; Edge Perception
      </div>
      <div class="auto-header-badges">
        <button class="mode-pill active" id="btn-manual-mode" onclick="setControlMode('manual')">GAMEPAD</button>
        <button class="mode-pill" id="btn-ai-mode" onclick="setControlMode('ai')">AI CONTROL</button>
        <span class="mode-pill" id="auto-mode-badge">MODE: ON-BOARD AI</span>
        <span class="server-pill" id="auto-server-badge" style="display:none;">EDGE SERVER</span>
        <button class="auto-toggle-btn" id="btn-auto-toggle" onclick="toggleAutoDrive()">
          <span class="toggle-switch-dot" id="auto-switch-dot"></span>
          <span id="auto-toggle-label">ENGAGE AUTO DRIVE</span>
        </button>
      </div>
    </div>

    <!-- Active Action Decision Banner -->
    <div class="auto-action-bar" id="auto-action-bar">
      <div class="auto-action-badge" id="auto-action-badge">STANDBY</div>
      <div class="auto-action-reason" id="auto-action-reason">Manual driving control active. Press 'Engage Auto Drive' or press Key [Z].</div>
      <div class="auto-action-metrics">
        <span class="auto-badge-pill" id="auto-pose-badge" style="color:var(--accent-cyan);">POSE: (0,0) 0°</span>
        <span class="auto-badge-pill" id="auto-sonar-badge" style="color:var(--accent-cyan);font-weight:700;">SONAR: -- cm</span>
        <span class="auto-badge-pill" id="auto-fps-badge">0.0 FPS</span>
        <span class="auto-badge-pill" id="auto-la-badge" style="color:var(--accent-cyan);font-weight:700;">LANE: ON</span>
        <span class="auto-badge-pill" id="auto-tr-badge" style="color:var(--accent-cyan);font-weight:700;">RULES: ON</span>
        <span class="auto-badge-pill" id="auto-haz-badge" style="color:var(--accent-green);">CLEAR</span>
      </div>
    </div>
    <div id="auto-obs-pose-banner" style="display:none;margin-top:6px;padding:6px 12px;border-radius:8px;background:rgba(255,176,32,0.12);border:1px solid rgba(255,176,32,0.3);color:var(--accent-amber);font-size:11px;font-weight:700;letter-spacing:0.02em;align-items:center;gap:6px;font-family:var(--font-mono);">
      <span>⚠️ LAST OBSTACLE POSE:</span>
      <span id="auto-obs-pose-val">(0,0) 0°</span>
    </div>

    <!-- Control Strips: Driving Modes, Clearance, Speed, and Server AI -->
    <div class="auto-grid-controls">
      <!-- 1. Mode Selector -->
      <div class="auto-ctrl-box">
        <div class="auto-k">Driving Mode</div>
        <div class="mode-btn-group">
          <button class="mode-btn" id="btn-mode-hybrid" onclick="setAutoMode('hybrid')">Hybrid</button>
          <button class="mode-btn active" id="btn-mode-local" onclick="setAutoMode('local_ai')">Local AI</button>
          <button class="mode-btn" id="btn-mode-server" onclick="setAutoMode('server_ai')">Server AI</button>
          <button class="mode-btn" id="btn-mode-paid" onclick="setAutoMode('paid_ai')">Paid Vision</button>
        </div>
      </div>

      <!-- 2. Path Clearance Discovery -->
      <div class="auto-ctrl-box">
        <div class="auto-k">Path Clearance Discovery</div>
        <div class="clearance-row">
          <div class="clearance-item">
            <span style="width:38px;">Left:</span>
            <div class="clearance-bar-wrap"><div class="clearance-bar" id="bar-left" style="width:100%"></div></div>
            <b id="val-clear-left">100%</b>
          </div>
          <div class="clearance-item">
            <span style="width:38px;">Right:</span>
            <div class="clearance-bar-wrap"><div class="clearance-bar" id="bar-right" style="width:100%"></div></div>
            <b id="val-clear-right">100%</b>
          </div>
        </div>
        <button class="tool-btn" onclick="triggerPathScan()" title="Pan camera servo to evaluate left vs right path clearance" style="margin-top:8px;justify-content:center;">
          ⟳ Scan Clearance Sweep
        </button>
      </div>

      <!-- Ultrasonic Sensor Real-Time Distance Widget -->
      <div class="auto-ctrl-box">
        <div class="auto-k">Ultrasonic Radar (HC-SR04)</div>
        <div style="display:flex;align-items:center;justify-content:space-between;margin-top:4px;">
          <div style="display:flex;align-items:baseline;gap:4px;">
            <span id="sonar-dist-large" style="font-size:26px;font-weight:800;color:var(--accent-cyan);font-family:var(--font-mono);">--</span>
            <span style="font-size:12px;color:var(--text-muted);font-weight:600;font-family:var(--font-mono);">cm</span>
          </div>
          <span id="sonar-status-badge" style="font-size:10px;font-weight:700;padding:3px 8px;border-radius:6px;background:rgba(0,229,153,0.15);color:var(--accent-green);border:1px solid rgba(0,229,153,0.3);font-family:var(--font-mono);">
            CLEAR
          </span>
        </div>
        <div class="clearance-bar-wrap" style="margin-top:8px;height:8px;">
          <div class="clearance-bar" id="sonar-meter-bar" style="width:100%;background:var(--accent-green);transition:width 0.15s ease, background 0.2s ease;"></div>
        </div>
        <div style="display:flex;justify-content:space-between;margin-top:6px;font-size:9px;color:var(--text-muted);font-weight:600;font-family:var(--font-mono);">
          <span>0cm (Hit)</span>
          <span>12cm (Rev)</span>
          <span>20cm (Stop)</span>
          <span>35cm (Warn)</span>
          <span>&gt;100cm</span>
        </div>
      </div>

      <!-- 3. Cruise Speed Limit -->
      <div class="auto-ctrl-box">
        <div class="auto-k">Cruise Speed Limit &bull; <b id="val-cruise-spd" style="color:var(--accent-cyan);font-family:var(--font-mono);">55%</b></div>
        <input type="range" id="sl-cruise" min="25" max="100" value="55" oninput="updateCruiseSpeed(this.value)" style="--pct:55%;margin-top:10px;">
        <div style="display:flex;justify-content:space-between;font-size:9.5px;color:var(--text-muted);font-weight:600;">
          <span>Crawl (25%)</span><span>Cruising</span><span>Fast (100%)</span>
        </div>
      </div>

      <!-- 4. Edge Server IP & Overlay -->
      <div class="auto-ctrl-box">
        <div class="auto-k">Edge AI Server &amp; HUD</div>
        <div style="display:flex;gap:6px;margin-top:8px;">
          <input type="text" id="input-server-url" value="http://127.0.0.1:8000" placeholder="http://192.168.1.X:8000" class="server-input">
          <button class="tool-btn" onclick="saveServerUrl()">Connect</button>
        </div>
        <div style="display:flex;gap:6px;margin-top:6px;">
          <button class="tool-btn active" id="btn-toggle-hud" onclick="toggleHudOverlay()" style="flex:1;justify-content:center;">
            AI HUD Overlay: ON
          </button>
        </div>
      </div>

      <!-- 5. Map Route Points -->
      <div class="auto-ctrl-box">
        <div class="auto-k">Map Route Points</div>
        <div style="font-size:9.5px;color:var(--text-secondary);margin-top:5px;line-height:1.35;">Click the spatial map above to add Point 1, Point 2, and so on. Points are stored in click order.</div>
        <div style="display:flex;gap:6px;margin-top:8px;">
          <button class="tool-btn" onclick="clearMapRoute(event)" style="flex:1;justify-content:center;">Clear Map Route</button>
        </div>
        <div id="destination-status" style="font-size:9px;color:var(--text-muted);margin-top:6px;font-family:var(--font-mono);">No destinations registered</div>
      </div>

      <!-- 6. Real-World GPS Target Navigation -->
      <div class="auto-ctrl-box">
        <div style="display:flex;align-items:center;justify-content:space-between;">
          <div class="auto-k">GPS Waypoint Navigation</div>
          <span style="font-size:8.5px;font-weight:700;color:var(--text-muted);background:rgba(255,255,255,0.06);padding:2px 6px;border-radius:4px;font-family:var(--font-mono);">WGS-84</span>
        </div>
        <div style="display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:8px;">
          <input type="number" step="0.000001" id="gps-target-lat" placeholder="Target Latitude" class="server-input">
          <input type="number" step="0.000001" id="gps-target-lon" placeholder="Target Longitude" class="server-input">
        </div>
        <div style="display:flex;gap:6px;margin-top:6px;">
          <button class="tool-btn" onclick="plotGpsTarget()" style="flex:1;justify-content:center;">Plot Target</button>
          <button class="tool-btn" onclick="useDeviceGpsForOrigin()" title="Set Car GPS Origin from browser geolocation" style="justify-content:center;">Device GPS</button>
        </div>
        <div id="gps-live-display" style="font-size:9px;color:var(--text-muted);margin-top:6px;line-height:1.3;font-family:var(--font-mono);">
          Estimated GPS: <b>9.591600, 76.522200</b> (No onboard GPS chip; device geolocation relayed)
        </div>
      </div>

      <!-- 7. Driving Assists & Traffic Rules -->
      <div class="auto-ctrl-box">
        <div style="display:flex;align-items:center;justify-content:space-between;">
          <div class="auto-k">Assists &amp; Traffic Code</div>
          <span style="font-size:8.5px;font-weight:700;color:var(--text-muted);background:rgba(255,255,255,0.06);padding:2px 6px;border-radius:4px;font-family:var(--font-mono);">LKA &bull; CODE</span>
        </div>
        <div style="display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:8px;">
          <button class="tool-btn active" id="btn-toggle-lane-assist" onclick="toggleLaneAssist()" style="justify-content:center;">
            Lane Assist: ON
          </button>
          <button class="tool-btn active" id="btn-toggle-traffic-rules" onclick="toggleTrafficRules()" style="justify-content:center;">
            Traffic Rules: ON
          </button>
        </div>
        <div id="assists-status-hint" style="font-size:9px;color:var(--text-muted);margin-top:6px;line-height:1.35;">
          LKA boundary departure prevention &bull; Red light &amp; stop sign recognition
        </div>
      </div>
    </div>
  </div>

  <!-- 6. CONTROLS COCKPIT ROW -->
  <div class="card a-ctrl">

    <!-- D-Pad (Desktop) -->
    <div class="dpad-section desktop-only">
      <div class="lbl"><div class="lbl-dot"></div>Tactile D-Pad &bull; WASD / Arrow Keys</div>
      <div class="dpad-wrap">
        <div class="dpad">
          <div></div>
          <button class="d-btn" id="dpad-up" onpointerdown="hold('forward')" onpointerup="rel()" onpointercancel="rel()" title="Forward (W, A, or Up Arrow)">&#9650;<span class="key-hint">W/A</span></button>
          <div></div>
          <button class="d-btn" id="dpad-left" onpointerdown="hold('left')" onpointerup="rel()" onpointercancel="rel()" title="Left (Left Arrow)">&#9664;<span class="key-hint">&larr;</span></button>
          <button class="d-btn d-stp" id="dpad-stop" onpointerdown="rel()" title="Emergency Stop (Spacebar)">&#9632;<span class="key-hint">SPACE</span></button>
          <button class="d-btn" id="dpad-right" onpointerdown="hold('right')" onpointerup="rel()" onpointercancel="rel()" title="Right (Right Arrow)">&#9654;<span class="key-hint">&rarr;</span></button>
          <div></div>
          <button class="d-btn" id="dpad-down" onpointerdown="hold('backward')" onpointerup="rel()" onpointercancel="rel()" title="Reverse (S, D, or Down Arrow)">&#9660;<span class="key-hint">S/D</span></button>
          <div></div>
        </div>
      </div>
      <div style="font-size:10.5px;color:var(--text-secondary);text-align:center;margin-top:4px;line-height:1.4;font-weight:600;font-family:var(--font-mono);">
        <b>W/A</b> Fwd &bull; <b>S/D</b> Rev &bull; <b>&larr;/&rarr;</b> Turn &bull; <b>Space</b> Stop<br><b>Q / E</b> Pan Gimbal &bull; <b>C</b> Center
      </div>

      <!-- Wheel Direction Calibration & Channel Diagnostics -->
      <div style="background:var(--bg-surface);border:1px solid var(--border);border-radius:var(--r-sm);padding:10px 12px;margin-top:10px;">
        <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:8px;">
          <span style="font-size:10px;font-weight:700;letter-spacing:0.06em;color:var(--text-secondary);text-transform:uppercase;">Wheel Polarity Calibration</span>
          <span id="motor-pol-badge" style="font-size:9.5px;font-weight:700;color:var(--accent-cyan);font-family:var(--font-mono);">L: -1 | R: -1</span>
        </div>
        <div style="display:grid;grid-template-columns:1fr 1fr;gap:6px;">
          <button class="tool-btn" style="justify-content:center;font-size:10.5px;padding:6px;" onclick="flipMotor('left')" title="Invert Left Wheels forward/reverse polarity">
            ⟲ Flip Left
          </button>
          <button class="tool-btn" style="justify-content:center;font-size:10.5px;padding:6px;" onclick="flipMotor('right')" title="Invert Right Wheels forward/reverse polarity">
            ⟲ Flip Right
          </button>
        </div>
        <div style="display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:6px;">
          <button class="tool-btn" style="justify-content:center;font-size:10.5px;padding:6px;" onclick="testMotor('left_fwd')" title="Spin Left wheels forward for 1 second">
            ▶ Test Left
          </button>
          <button class="tool-btn" style="justify-content:center;font-size:10.5px;padding:6px;" onclick="testMotor('right_fwd')" title="Spin Right wheels forward for 1 second">
            ▶ Test Right
          </button>
        </div>
        <div style="margin-top:10px;border-top:1px solid var(--border);padding-top:8px;">
          <div style="font-size:9px;color:var(--text-muted);font-weight:700;line-height:1.4;font-family:var(--font-mono);">
            L298N CHANNEL TEST &bull; lift wheels clear of ground<br>
            IN1/2 = GPIO 17/18 &bull; IN3/4 = GPIO 22/23
          </div>
          <div style="display:grid;grid-template-columns:repeat(4,1fr);gap:4px;margin-top:6px;">
            <button class="tool-btn" style="justify-content:center;font-size:9px;padding:5px 2px;" onclick="testDriverChannel('left_fwd')">L FWD</button>
            <button class="tool-btn" style="justify-content:center;font-size:9px;padding:5px 2px;" onclick="testDriverChannel('left_rev')">L REV</button>
            <button class="tool-btn" style="justify-content:center;font-size:9px;padding:5px 2px;" onclick="testDriverChannel('right_fwd')">R FWD</button>
            <button class="tool-btn" style="justify-content:center;font-size:9px;padding:5px 2px;" onclick="testDriverChannel('right_rev')">R REV</button>
          </div>
          <div id="driver-test-result" style="font-size:9px;color:var(--text-muted);margin-top:6px;line-height:1.35;font-family:var(--font-mono);">Select one channel. Each pulse stops automatically.</div>
        </div>
      </div>

      <!-- Desktop Capture Buttons under D-Pad -->
      <div class="cap-row" style="margin-top:10px;">
        <button class="btn-cap btn-photo" onclick="capturePhoto()">
          <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M23 19a2 2 0 0 1-2 2H3a2 2 0 0 1-2-2V8a2 2 0 0 1 2-2h4l2-3h6l2 3h4a2 2 0 0 1 2 2z"/><circle cx="12" cy="13" r="4"/></svg>
          Photo
        </button>
        <button class="btn-cap btn-video" id="btn-vid" onclick="toggleVideo()">
          <span id="vid-ico">
            <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M23 7l-7 5 7 5V7z"/><rect x="1" y="5" width="15" height="14" rx="2" ry="2"/></svg>
          </span>&nbsp;<span id="vid-lbl">Record</span>
        </button>
      </div>
      <div class="rec-bar" id="rec-bar"><div class="rec-bar-dot"></div><span id="rec-t">00:00</span></div>
      <div class="cap-msg" id="cap-msg">Ready</div>
      <div class="gallery-wrap">
        <div class="gallery-hd" style="display:flex;align-items:center;gap:6px;">
          <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="3" y="3" width="18" height="18" rx="2" ry="2"/><circle cx="8.5" cy="8.5" r="1.5"/><polyline points="21 15 16 10 5 21"/></svg>
          Capture Gallery
        </div>
        <div class="gallery" id="gallery"><div class="g-empty">No captures yet</div></div>
      </div>
    </div>

    <!-- Precision Analog Joystick (Center) -->
    <div class="joy-section">
      <div class="lbl"><div class="lbl-dot"></div>Proportional Virtual Thumbstick</div>
      <canvas id="jc" width="240" height="240"></canvas>
      <div class="joy-meta">
        <div class="joy-tag">X:&nbsp;<b id="vx">0.00</b></div>
        <div class="joy-tag">Y:&nbsp;<b id="vy">0.00</b></div>
        <div class="joy-tag" id="st-badge">STOPPED</div>
      </div>
    </div>

    <!-- Capture Right Column (Desktop) -->
    <div class="cap-section desktop-only">
      <div class="lbl"><div class="lbl-dot"></div>Emergency Action &bull; Media</div>
      <button class="btn-stop" onclick="rel()">
        <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor"><rect x="4" y="4" width="16" height="16" rx="2"/></svg>
        EMERGENCY STOP
      </button>
      <div style="height:6px"></div>
      <div class="cap-row">
        <button class="btn-cap btn-photo" onclick="capturePhoto()">
          <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M23 19a2 2 0 0 1-2 2H3a2 2 0 0 1-2-2V8a2 2 0 0 1 2-2h4l2-3h6l2 3h4a2 2 0 0 1 2 2z"/><circle cx="12" cy="13" r="4"/></svg>
          Photo
        </button>
        <button class="btn-cap btn-video" id="btn-vid2" onclick="toggleVideo()">
          <span id="vid-ico2">
            <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M23 7l-7 5 7 5V7z"/><rect x="1" y="5" width="15" height="14" rx="2" ry="2"/></svg>
          </span>&nbsp;<span id="vid-lbl2">Record</span>
        </button>
      </div>
      <div class="rec-bar" id="rec-bar2"><div class="rec-bar-dot"></div><span id="rec-t2">00:00</span></div>
      <div class="cap-msg" id="cap-msg2">Ready</div>
    </div>

  </div><!-- /a-ctrl -->

</div><!-- /layout -->

<!-- ── MOBILE DOCK (TOUCHSCREEN DEVICES) ── -->
<div class="mobile-dock">
  <button class="btn-stop" onclick="rel()">
    <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor"><rect x="4" y="4" width="16" height="16" rx="2"/></svg>
    EMERGENCY STOP
  </button>

  <div class="cap-row">
    <button class="btn-cap btn-photo" onclick="capturePhoto()">
      <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M23 19a2 2 0 0 1-2 2H3a2 2 0 0 1-2-2V8a2 2 0 0 1 2-2h4l2-3h6l2 3h4a2 2 0 0 1 2 2z"/><circle cx="12" cy="13" r="4"/></svg>
      Photo
    </button>
    <button class="btn-cap btn-video" id="btn-vid-m" onclick="toggleVideo()">
      <span id="vid-ico-m">
        <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M23 7l-7 5 7 5V7z"/><rect x="1" y="5" width="15" height="14" rx="2" ry="2"/></svg>
      </span>&nbsp;<span id="vid-lbl-m">Record</span>
    </button>
  </div>
  <div class="rec-bar" id="rec-bar-m"><div class="rec-bar-dot"></div><span id="rec-t-m">00:00</span></div>
  <div class="cap-msg" id="cap-msg-m">Ready</div>

  <!-- Mobile Servo Pan & Calibration -->
  <div class="card" style="padding:14px;margin-top:6px;">
    <div class="lbl" style="margin-bottom:10px;"><div class="lbl-dot"></div>Camera Gimbal &bull; GPIO 12</div>
    <div class="sv-top" style="margin-bottom:8px;">
      <div>
        <div class="sv-num"><span id="sdeg-m">90</span><sup>°</sup></div>
        <div class="sv-lbl">Pan Angle &bull; Trim: <b id="strim-m">+0.0°</b></div>
      </div>
    </div>
    <input type="range" id="sl-m" min="25" max="155" value="90"
           oninput="setServo(this.value)" style="--pct:50%">
    <div class="sv-btns" style="margin-bottom:10px;">
      <button class="sv-btn sv-btn-sec" onclick="nudgeServo(-5)">-5°</button>
      <button class="sv-btn sv-btn-sec" onclick="nudgeServo(-1)">-1°</button>
      <button class="sv-btn" onclick="setServo(90)">Center</button>
      <button class="sv-btn sv-btn-sec" onclick="nudgeServo(1)">+1°</button>
      <button class="sv-btn sv-btn-sec" onclick="nudgeServo(5)">+5°</button>
    </div>
    <div style="display:flex;gap:6px;">
      <button class="sv-btn sv-btn-sec" style="font-size:11px;height:32px;" onclick="calibrateCurrentAsCenter()">Set Center</button>
      <button class="sv-btn sv-btn-sec" style="font-size:11px;height:32px;color:var(--text-muted);" onclick="resetServoTrim()">Reset Trim</button>
      <button class="sv-btn sv-btn-sec" style="font-size:11px;height:32px;color:var(--accent-green);font-weight:700;" onclick="relaxServo()">Sleep / Relax</button>
    </div>
  </div>

  <!-- Mobile Saved Media Gallery -->
  <div class="card" style="padding:14px;margin-top:6px;">
    <div class="gallery-hd" style="display:flex;align-items:center;gap:6px;">
      <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="3" y="3" width="18" height="18" rx="2" ry="2"/><circle cx="8.5" cy="8.5" r="1.5"/><polyline points="21 15 16 10 5 21"/></svg>
      Saved Media Gallery
    </div>
    <div class="gallery" id="gallery-m"><div class="g-empty">No captures yet</div></div>
  </div>
</div>

<script>
{orig_js}
</script>
</body>
</html>
"""

    with open(template_path, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"Successfully generated redesigned template.html ({len(html)} bytes, {len(html.splitlines())} lines)")

    required_ids = [
        'cam', 'cam-grid', 'cam-hud-info', 'cam-sonar-badge', 'cam-sonar-txt',
        'target-pin-layer', 'pin-reticle', 'pin-tag', 'btn-clear-pin',
        'spatial-canvas-wrap', 'spatial-map-canvas', 'hud-car-pose', 'hud-servo-angle',
        'hud-sonar-pill', 'hud-sonar-val', 'hud-target-pill', 'hud-target-info',
        'gaze-lock-badge', 'btn-toggle-gaze', 'btn-clear-3d-target',
        'st-ico', 'st-name', 'ring', 'hx', 'hy', 'hs',
        'sdeg', 'sdeg-m', 'strim', 'strim-m', 'sl', 'sl-m', 'aknob', 'arc',
        'btn-manual-mode', 'btn-ai-mode', 'auto-mode-badge', 'auto-server-badge',
        'btn-auto-toggle', 'auto-switch-dot', 'auto-toggle-label', 'auto-action-bar',
        'auto-action-badge', 'auto-action-reason', 'auto-pose-badge', 'auto-sonar-badge',
        'auto-fps-badge', 'auto-la-badge', 'auto-tr-badge', 'auto-haz-badge',
        'auto-obs-pose-banner', 'auto-obs-pose-val', 'btn-mode-hybrid', 'btn-mode-local',
        'btn-mode-server', 'btn-mode-paid', 'bar-left', 'bar-right', 'val-clear-left',
        'val-clear-right', 'sonar-dist-large', 'sonar-status-badge', 'sonar-meter-bar',
        'sl-cruise', 'val-cruise-spd', 'input-server-url', 'btn-toggle-hud',
        'btn-quick-auto', 'btn-ai-overlay', 'btn-grid', 'btn-fliph', 'btn-flipv',
        'btn-rot', 'btn-fit', 'destination-status', 'gps-target-lat', 'gps-target-lon',
        'gps-live-display', 'btn-toggle-lane-assist', 'btn-toggle-traffic-rules',
        'assists-status-hint', 'dpad-up', 'dpad-down', 'dpad-left', 'dpad-right',
        'dpad-stop', 'motor-pol-badge', 'driver-test-result', 'jc', 'vx', 'vy',
        'st-badge', 'btn-vid', 'btn-vid2', 'btn-vid-m', 'vid-ico', 'vid-ico2',
        'vid-ico-m', 'vid-lbl', 'vid-lbl2', 'vid-lbl-m', 'rec-bar', 'rec-bar2',
        'rec-bar-m', 'rec-t', 'rec-t2', 'rec-t-m', 'cap-msg', 'cap-msg2',
        'cap-msg-m', 'gallery', 'gallery-m', 'sonar-pill', 'sonar-txt', 'wifi-pill',
        'wifi-txt', 'bt-pill', 'bt-txt', 'cpu-temp', 'control-mode-pill', 'control-mode-txt'
    ]
    missing = [i for i in required_ids if f'id="{i}"' not in html and f"id='{i}'" not in html]
    if missing:
        print(f"Warning: {len(missing)} IDs missing: {missing}")
    else:
        print(f"Verification PASSED: All {len(required_ids)} required DOM element IDs are present!")

if __name__ == "__main__":
    build_template()
