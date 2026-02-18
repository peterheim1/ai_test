"""Web interface for Robbie voice control server.

Provides:
  GET  /             — single-page HTML control panel
  POST /api/command  — inject text command (bypass wake word + STT)
  POST /api/tts_mute — set TTS mute state  {"muted": true/false}
  WS   /ws           — real-time event stream + command input
"""

import asyncio
import json
import logging
from collections import deque
from typing import Any

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Single-page HTML (embedded — no separate file to deploy)
# ---------------------------------------------------------------------------

_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Robbie</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{background:#0d1117;color:#e6edf3;font-family:'Courier New',monospace;font-size:14px}
    header{display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:8px;
           padding:12px 16px;background:#161b22;border-bottom:1px solid #30363d}
    .title{font-size:18px;font-weight:bold;color:#58a6ff}
    .status-badge{padding:4px 12px;border-radius:12px;font-size:12px;background:#21262d;color:#8b949e}
    .status-badge.listening{background:#1f2d1f;color:#3fb950}
    .status-badge.recording{background:#3d1f1f;color:#f85149}
    .status-badge.processing{background:#1f2a3d;color:#58a6ff}
    .status-badge.speaking{background:#2d1f3d;color:#bc8cff}
    .status-badge.disconnected{background:#3d2600;color:#e3b341}
    .mute-wrap{display:flex;align-items:center;gap:8px;cursor:pointer}
    .mute-label{font-size:12px;color:#8b949e;user-select:none}
    .toggle{position:relative;width:40px;height:20px;flex-shrink:0}
    .toggle input{opacity:0;width:0;height:0}
    .slider{position:absolute;cursor:pointer;inset:0;background:#21262d;border-radius:20px;transition:.3s}
    .slider:before{position:absolute;content:"";height:14px;width:14px;left:3px;bottom:3px;
                   background:#8b949e;border-radius:50%;transition:.3s}
    input:checked+.slider{background:#f85149}
    input:checked+.slider:before{transform:translateX(20px);background:#fff}
    main{padding:16px;display:flex;flex-direction:column;gap:14px;max-width:860px;margin:0 auto}
    .banner{color:#f85149;text-align:center;padding:6px;font-size:12px;display:none}
    .card{background:#161b22;border:1px solid #30363d;border-radius:8px;overflow:hidden}
    .card-hdr{padding:6px 12px;background:#21262d;font-size:11px;color:#8b949e;
              text-transform:uppercase;letter-spacing:1px}
    .card-body{padding:12px}
    .cmd-row{display:flex;gap:8px}
    .cmd-input{flex:1;background:#0d1117;border:1px solid #30363d;border-radius:6px;
               color:#e6edf3;font-family:inherit;font-size:14px;padding:8px 12px}
    .cmd-input:focus{outline:none;border-color:#58a6ff}
    .send-btn{background:#238636;color:#fff;border:none;border-radius:6px;
              padding:8px 16px;cursor:pointer;font-family:inherit;font-size:14px}
    .send-btn:hover{background:#2ea043}
    .grid{display:grid;grid-template-columns:80px 1fr;gap:5px 12px;align-items:start}
    .lbl{color:#8b949e;font-size:12px;padding-top:1px}
    .val{color:#e6edf3;word-break:break-word}
    .val.muted{color:#6e6e6e}
    .badge{font-size:11px;margin-left:6px}
    .badge.muted{color:#f85149}
    .badge.web{color:#58a6ff}
    .log-box{font-family:'Courier New',monospace;font-size:12px;height:300px;overflow-y:auto;
             background:#0d1117;padding:8px;line-height:1.7}
    .ll{white-space:pre-wrap;word-break:break-all}
    .ll.err{color:#f85149} .ll.warn{color:#e3b341} .ll.info{color:#6e7681}
    .ll.hear{color:#3fb950} .ll.intent{color:#58a6ff}
    .ll.tts{color:#bc8cff} .ll.tts-muted{color:#6e4496}
    .infobar{display:flex;align-items:center;gap:16px;flex-wrap:wrap;
             padding:5px 16px;background:#0d1117;border-bottom:1px solid #21262d;
             font-size:12px;color:#8b949e}
    .sep{color:#30363d}
  </style>
</head>
<body>
<header>
  <div class="title">&#x1F916; ROBBIE</div>
  <div id="status" class="status-badge disconnected">&#x25CF; CONNECTING</div>
  <label class="mute-wrap" title="Silent mode — text shown, robot stays quiet">
    <span class="mute-label">&#x1F507; Silent</span>
    <div class="toggle">
      <input type="checkbox" id="muteToggle" onchange="toggleMute()">
      <span class="slider"></span>
    </div>
  </label>
</header>
<div class="infobar">
  <span id="clock">--:--:--</span>
  <span class="sep">|</span>
  <span id="weather">fetching weather...</span>
</div>
<main>
  <div id="banner" class="banner">&#x26A0; Disconnected — reconnecting...</div>

  <div class="card">
    <div class="card-hdr">Command</div>
    <div class="card-body">
      <div class="cmd-row">
        <input id="cmdInput" class="cmd-input" type="text"
               placeholder="type a command and press Enter..."
               onkeydown="if(event.key==='Enter')sendCmd()">
        <button class="send-btn" onclick="sendCmd()">Send</button>
      </div>
    </div>
  </div>

  <div class="card">
    <div class="card-hdr">Last Interaction</div>
    <div class="card-body">
      <div class="grid">
        <span class="lbl">Heard</span>   <span id="lHeard"    class="val">&mdash;</span>
        <span class="lbl">Intent</span>  <span id="lIntent"   class="val">&mdash;</span>
        <span class="lbl">Response</span><span id="lResponse" class="val">&mdash;</span>
      </div>
    </div>
  </div>

  <div class="card">
    <div class="card-hdr">Live Log</div>
    <div id="log" class="log-box"></div>
  </div>
</main>
<script>
  let ws, reconnTimer;

  function connect() {
    ws = new WebSocket('ws://' + location.host + '/ws');
    ws.onopen = () => {
      document.getElementById('banner').style.display = 'none';
    };
    ws.onclose = () => {
      document.getElementById('banner').style.display = 'block';
      setStatus('disconnected');
      reconnTimer = setTimeout(connect, 3000);
    };
    ws.onmessage = (e) => {
      const d = JSON.parse(e.data);
      switch (d.type) {
        case 'status':
          setStatus(d.state);
          if (d.state === 'listening') addLog('· listening', 'info');
          break;
        case 'transcript': {
          const src = d.source === 'web' ? '<span class="badge web">[web]</span>' : '';
          document.getElementById('lHeard').innerHTML = '"' + esc(d.text) + '"' + src;
          addLog('HEAR  "' + d.text + '"' + (d.source === 'web' ? ' [web]' : ''), 'hear');
          break;
        }
        case 'intent': {
          const p = Object.entries(d.params || {}).map(([k,v]) => k + '=' + v).join('  ');
          document.getElementById('lIntent').textContent = d.name + (p ? '  ' + p : '');
          addLog('INTENT ' + d.name + (p ? '  ' + p : ''), 'intent');
          break;
        }
        case 'tts': {
          const badge = d.muted ? '<span class="badge muted">&#x1F507; muted</span>' : '';
          const el = document.getElementById('lResponse');
          el.innerHTML = '"' + esc(d.text) + '"' + badge;
          el.className = d.muted ? 'val muted' : 'val';
          addLog('TTS   "' + d.text + '"' + (d.muted ? ' [muted]' : ''), d.muted ? 'tts-muted' : 'tts');
          break;
        }
        case 'log':
          addLog(d.msg, d.level || 'info');
          break;
        case 'tts_mute':
          document.getElementById('muteToggle').checked = d.muted;
          break;
      }
    };
  }

  function setStatus(state) {
    const el = document.getElementById('status');
    const labels = {
      listening: '&#x25CF; LISTENING',
      recording: '&#x25CF; RECORDING',
      processing: '&#x25CC; PROCESSING',
      speaking:   '&#x25B6; SPEAKING',
      disconnected: '&#x25CF; DISCONNECTED',
    };
    el.innerHTML = labels[state] || state.toUpperCase();
    el.className = 'status-badge ' + state;
  }

  function addLog(msg, cls) {
    const box = document.getElementById('log');
    const line = document.createElement('div');
    const ts = new Date().toTimeString().slice(0, 8);
    line.className = 'll ' + (cls || 'info');
    line.textContent = ts + '  ' + msg;
    box.appendChild(line);
    box.scrollTop = box.scrollHeight;
    while (box.children.length > 200) box.removeChild(box.firstChild);
  }

  function sendCmd() {
    const inp = document.getElementById('cmdInput');
    const text = inp.value.trim();
    if (!text || !ws || ws.readyState !== 1) return;
    ws.send(JSON.stringify({type: 'command', text}));
    inp.value = '';
  }

  function toggleMute() {
    const muted = document.getElementById('muteToggle').checked;
    if (ws && ws.readyState === 1)
      ws.send(JSON.stringify({type: 'set_tts_mute', muted}));
  }

  function esc(s) {
    return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
  }

  function updateClock() {
    const n = new Date();
    const t = n.toTimeString().slice(0, 8);
    const d = n.toLocaleDateString('en-GB', {weekday:'short', day:'numeric', month:'short'});
    document.getElementById('clock').textContent = t + '  \u00b7  ' + d;
  }
  updateClock();
  setInterval(updateClock, 1000);

  async function fetchWeather() {
    try {
      const r = await fetch('https://wttr.in/?format=%t+%C&lang=en');
      const txt = r.ok ? (await r.text()).trim() : '';
      document.getElementById('weather').textContent = txt || '';
    } catch(e) {
      document.getElementById('weather').textContent = '';
    }
  }
  fetchWeather();
  setInterval(fetchWeather, 600000);

  connect();
</script>
</body>
</html>"""


# ---------------------------------------------------------------------------
# WebServer class
# ---------------------------------------------------------------------------

class WebServer:
    """FastAPI/WebSocket server for the Robbie control panel."""

    def __init__(self, host: str = "0.0.0.0", port: int = 8080):
        self._host = host
        self._port = port
        self._tts_muted: bool = False
        self._clients: set = set()
        self._log_buffer: deque = deque(maxlen=100)
        self._voice_server = None

    @property
    def tts_muted(self) -> bool:
        return self._tts_muted

    async def broadcast(self, event: dict):
        """Push a JSON event to all connected WebSocket clients."""
        self._log_buffer.append(event)
        dead = set()
        msg = json.dumps(event)
        for ws in self._clients:
            try:
                await ws.send_text(msg)
            except Exception:
                dead.add(ws)
        self._clients -= dead

    async def start(self, voice_server):
        """Start the web server on the current asyncio event loop."""
        try:
            from fastapi import FastAPI, WebSocket, WebSocketDisconnect
            from fastapi.responses import HTMLResponse
            import uvicorn
        except ImportError:
            logger.error(
                "fastapi/uvicorn not installed — web interface disabled. "
                "Run: pip install fastapi uvicorn"
            )
            return

        self._voice_server = voice_server
        app = FastAPI()

        @app.get("/", response_class=HTMLResponse)
        async def index():
            return _HTML

        @app.post("/api/command")
        async def api_command(body: dict[str, Any]):
            text = body.get("text", "").strip()
            if text and self._voice_server:
                asyncio.create_task(self._voice_server.handle_text_command(text))
            return {"status": "ok"}

        @app.post("/api/tts_mute")
        async def api_tts_mute(body: dict[str, Any]):
            self._tts_muted = bool(body.get("muted", False))
            await self.broadcast({"type": "tts_mute", "muted": self._tts_muted})
            return {"status": "ok", "muted": self._tts_muted}

        @app.websocket("/ws")
        async def ws_endpoint(websocket: WebSocket):
            await websocket.accept()
            self._clients.add(websocket)
            # Replay buffered events to new client so log is populated on load
            for event in self._log_buffer:
                try:
                    await websocket.send_text(json.dumps(event))
                except Exception:
                    break
            # Send current mute state
            try:
                await websocket.send_text(
                    json.dumps({"type": "tts_mute", "muted": self._tts_muted})
                )
            except Exception:
                pass
            try:
                while True:
                    data = await websocket.receive_text()
                    try:
                        msg = json.loads(data)
                    except json.JSONDecodeError:
                        continue
                    if msg.get("type") == "command":
                        text = msg.get("text", "").strip()
                        if text and self._voice_server:
                            asyncio.create_task(
                                self._voice_server.handle_text_command(text)
                            )
                    elif msg.get("type") == "set_tts_mute":
                        self._tts_muted = bool(msg.get("muted", False))
                        await self.broadcast(
                            {"type": "tts_mute", "muted": self._tts_muted}
                        )
            except WebSocketDisconnect:
                pass
            except Exception as e:
                logger.debug(f"WebSocket error: {e}")
            finally:
                self._clients.discard(websocket)

        config = uvicorn.Config(
            app,
            host=self._host,
            port=self._port,
            loop="none",
            log_level="warning",
        )
        server = uvicorn.Server(config)
        server.install_signal_handlers = lambda: None  # don't override main handlers
        logger.info(f"Web interface at http://{self._host}:{self._port}")
        asyncio.create_task(server.serve())
