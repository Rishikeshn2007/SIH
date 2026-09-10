/**
 * app.js — UGV Navigation Dashboard (vanilla JS, no framework)
 *
 * Coordinate convention used in this file:
 *   row / y : 0-based row, 0 = top
 *   col / x : 0-based col, 0 = left
 *
 *   API sends [row, col] arrays.
 *   ESP32 / display uses x = col, y = row.
 */

"use strict";

// ─── Constants ────────────────────────────────────────────────────────────────

const POLL_INTERVAL_MS = 750;

// Canvas rendering colours
const COLOR = {
  free:        "#1c2330",
  obstacle:    "#f85149",
  route:       "#58a6ff",
  waypoint:    "#1e4a8a",
  vehicle:     "#e3b341",
  start:       "#3fb950",
  goal:        "#f85149",
  grid_line:   "#21262d",
};

// ─── State ────────────────────────────────────────────────────────────────────

let _matrix   = null;   // 2-D array of 0/1
let _rows     = 0;
let _cols     = 0;
let _route    = [];     // list of [row, col]
let _status   = {};     // last /navigation/status response
let _pollTimer = null;

// ─── DOM refs ─────────────────────────────────────────────────────────────────

const canvas      = document.getElementById("mapCanvas");
const ctx         = canvas.getContext("2d");

const srcRowEl    = document.getElementById("srcRow");
const srcColEl    = document.getElementById("srcCol");
const dstRowEl    = document.getElementById("dstRow");
const dstColEl    = document.getElementById("dstCol");

const btnStart    = document.getElementById("btnStart");
const btnStop     = document.getElementById("btnStop");

const elStatus    = document.getElementById("navStatus");
const elPos       = document.getElementById("curPos");
const elHeading   = document.getElementById("curHeading");
const elCommand   = document.getElementById("curCommand");
const elSpeed     = document.getElementById("curSpeed");
const elProgress  = document.getElementById("routeProgress");
const elProgressBar = document.getElementById("progressBar");
const elRouteLen  = document.getElementById("routeLen");

const logRx       = document.getElementById("logRx");
const logTx       = document.getElementById("logTx");

// ─── Startup ──────────────────────────────────────────────────────────────────

window.addEventListener("DOMContentLoaded", () => {
  btnStart.addEventListener("click", onStartNav);
  btnStop.addEventListener("click",  onStopNav);
  loadMap();
});

// ─── Map loading ──────────────────────────────────────────────────────────────

async function loadMap() {
  try {
    const res  = await fetch("/map");
    const body = await res.json();
    if (body.status !== "ok") {
      console.warn("Map not ready:", body.message);
      return;
    }
    _matrix = body.data.matrix;
    _rows   = body.data.rows;
    _cols   = body.data.cols;
    resizeCanvas();
    redraw();
  } catch (e) {
    console.error("Failed to load map:", e);
  }
}

function resizeCanvas() {
  if (!_rows || !_cols) return;
  const maxPx  = Math.min(window.innerWidth - 340, window.innerHeight - 120, 720);
  const cellPx = Math.max(Math.floor(maxPx / Math.max(_rows, _cols)), 8);
  canvas.width  = _cols * cellPx;
  canvas.height = _rows * cellPx;
  canvas._cellPx = cellPx;
}

// ─── Navigation start ─────────────────────────────────────────────────────────

async function onStartNav() {
  const srcRow = parseInt(srcRowEl.value, 10);
  const srcCol = parseInt(srcColEl.value, 10);
  const dstRow = parseInt(dstRowEl.value, 10);
  const dstCol = parseInt(dstColEl.value, 10);

  if ([srcRow, srcCol, dstRow, dstCol].some(isNaN)) {
    alert("Please enter valid integer coordinates.");
    return;
  }

  btnStart.disabled = true;

  try {
    const res = await fetch("/navigation/start", {
      method:  "POST",
      headers: { "Content-Type": "application/json" },
      body:    JSON.stringify({
        source:      [srcRow, srcCol],
        destination: [dstRow, dstCol],
      }),
    });

    const body = await res.json();

    if (body.status !== "ok") {
      alert("Navigation start failed:\n" + (body.message || JSON.stringify(body)));
      btnStart.disabled = false;
      return;
    }

    _route = body.data.route || [];
    startPolling();
    btnStop.disabled = false;

  } catch (e) {
    alert("Request failed: " + e);
    btnStart.disabled = false;
  }
}

// ─── Navigation stop ──────────────────────────────────────────────────────────

async function onStopNav() {
  stopPolling();
  try {
    await fetch("/navigation/stop", { method: "POST" });
  } catch (_) {}
  btnStart.disabled = false;
  btnStop.disabled  = true;
  setStatusBadge("IDLE");
}

// ─── Polling ──────────────────────────────────────────────────────────────────

function startPolling() {
  stopPolling();
  _pollTimer = setInterval(poll, POLL_INTERVAL_MS);
  poll();  // immediate first tick
}

function stopPolling() {
  if (_pollTimer) { clearInterval(_pollTimer); _pollTimer = null; }
}

async function poll() {
  try {
    const res  = await fetch("/navigation/status");
    const body = await res.json();
    if (body.status !== "ok") return;
    _status = body.data;

    // Update route from status if available
    if (_status.route && _status.route.length) {
      _route = _status.route;
    }

    updateSidebar(_status);
    updatePacketLog(_status.packet_log || []);
    redraw();

    // Auto-stop polling when completed or idle
    if (_status.status === "COMPLETED" || _status.status === "IDLE") {
      stopPolling();
      btnStart.disabled = false;
      btnStop.disabled  = true;
    }

  } catch (e) {
    console.warn("Poll error:", e);
  }
}

// ─── Sidebar updates ──────────────────────────────────────────────────────────

function updateSidebar(s) {
  setStatusBadge(s.status || "IDLE");

  const pos = s.current_position;
  elPos.textContent = pos
    ? `(${pos[0]}, ${pos[1]})  x=${pos[1]}  y=${pos[0]}`
    : "—";

  elHeading.textContent = headingLabel(s.current_heading);

  const cmd = s.last_command;
  if (cmd) {
    elCommand.textContent = cmd.command || "—";
    elSpeed.textContent   = (cmd.speed !== undefined ? cmd.speed : "—") + " / 200";
    styleCommand(cmd.command);
  }

  // Progress
  const idx = s.current_route_index || 0;
  const len = s.route_length || 0;
  const pct = len > 1 ? Math.round(((idx + 1) / len) * 100) : 0;
  elProgress.textContent = len ? `${idx + 1} / ${len}  (${pct}%)` : "—";
  elProgressBar.style.width = pct + "%";
  elRouteLen.textContent = len ? `${len} waypoints` : "—";
}

function setStatusBadge(label) {
  const map = {
    "IDLE":      "badge-idle",
    "RUNNING":   "badge-running",
    "COMPLETED": "badge-completed",
    "ERROR":     "badge-error",
  };
  elStatus.textContent = label;
  elStatus.className   = "badge " + (map[label] || "badge-idle");
}

function headingLabel(deg) {
  const labels = { 0: "0° N ↑", 90: "90° E →", 180: "180° S ↓", 270: "270° W ←" };
  return (deg !== undefined && deg !== null) ? (labels[deg] || deg + "°") : "—";
}

function styleCommand(cmd) {
  const el = elCommand;
  const colorMap = {
    "move_forward":  "#3fb950",
    "move_backward": "#d29922",
    "turn_left":     "#58a6ff",
    "turn_right":    "#58a6ff",
    "stop":          "#f85149",
  };
  el.style.color = colorMap[cmd] || "#e6edf3";
}

// ─── Packet log ───────────────────────────────────────────────────────────────

function updatePacketLog(log) {
  const rx = log.filter(e => e.direction === "rx").slice(-8);
  const tx = log.filter(e => e.direction === "tx").slice(-8);
  renderLogColumn(logRx, rx);
  renderLogColumn(logTx, tx);
}

function renderLogColumn(el, entries) {
  el.innerHTML = "";
  entries.reverse().forEach(e => {
    const div = document.createElement("div");
    div.className = "log-entry";
    div.textContent = JSON.stringify(e.data, null, 2);
    el.appendChild(div);
  });
}

// ─── Canvas rendering ─────────────────────────────────────────────────────────

function redraw() {
  if (!_matrix || !canvas._cellPx) return;

  const cp = canvas._cellPx;
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // ── Draw cells ────────────────────────────────────────────────────────
  for (let r = 0; r < _rows; r++) {
    for (let c = 0; c < _cols; c++) {
      ctx.fillStyle = _matrix[r][c] === 1 ? COLOR.obstacle : COLOR.free;
      ctx.fillRect(c * cp, r * cp, cp, cp);
    }
  }

  // ── Grid lines ────────────────────────────────────────────────────────
  ctx.strokeStyle = COLOR.grid_line;
  ctx.lineWidth   = 0.5;
  for (let r = 0; r <= _rows; r++) {
    ctx.beginPath(); ctx.moveTo(0, r * cp); ctx.lineTo(_cols * cp, r * cp); ctx.stroke();
  }
  for (let c = 0; c <= _cols; c++) {
    ctx.beginPath(); ctx.moveTo(c * cp, 0); ctx.lineTo(c * cp, _rows * cp); ctx.stroke();
  }

  // ── Route ─────────────────────────────────────────────────────────────
  if (_route.length > 1) {
    const idx = (_status && _status.current_route_index) || 0;

    // Visited portion (dim)
    if (idx > 0) {
      ctx.strokeStyle = "#1a3a6a";
      ctx.lineWidth   = Math.max(2, cp * 0.18);
      ctx.lineCap     = "round";
      ctx.lineJoin    = "round";
      ctx.beginPath();
      _route.slice(0, idx + 1).forEach(([r, c], i) => {
        const x = c * cp + cp / 2, y = r * cp + cp / 2;
        i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
      });
      ctx.stroke();
    }

    // Remaining portion (bright)
    ctx.strokeStyle = COLOR.route;
    ctx.lineWidth   = Math.max(2, cp * 0.18);
    ctx.lineCap     = "round";
    ctx.lineJoin    = "round";
    ctx.beginPath();
    _route.slice(idx).forEach(([r, c], i) => {
      const x = c * cp + cp / 2, y = r * cp + cp / 2;
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.stroke();
  }

  // ── Source marker ─────────────────────────────────────────────────────
  const src = _status && _status.source;
  if (src) drawMarker(src[0], src[1], cp, COLOR.start, "S");

  // ── Goal marker ───────────────────────────────────────────────────────
  const dst = _status && _status.destination;
  if (dst) drawMarker(dst[0], dst[1], cp, COLOR.goal, "G");

  // ── Vehicle ───────────────────────────────────────────────────────────
  const pos = _status && _status.current_position;
  if (pos) {
    const hdg = (_status && _status.current_heading) || 0;
    drawVehicle(pos[0], pos[1], cp, hdg);
  }
}

function drawMarker(row, col, cp, color, label) {
  const x = col * cp + cp / 2;
  const y = row * cp + cp / 2;
  const r = Math.max(5, cp * 0.38);

  ctx.beginPath();
  ctx.arc(x, y, r, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();

  ctx.strokeStyle = "#fff";
  ctx.lineWidth   = 1.5;
  ctx.stroke();

  if (cp > 14) {
    ctx.fillStyle   = "#fff";
    ctx.font        = `bold ${Math.max(8, cp * 0.3)}px Inter`;
    ctx.textAlign   = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(label, x, y);
  }
}

function drawVehicle(row, col, cp, headingDeg) {
  const x = col * cp + cp / 2;
  const y = row * cp + cp / 2;
  const r = Math.max(4, cp * 0.32);

  // Body
  ctx.beginPath();
  ctx.arc(x, y, r, 0, Math.PI * 2);
  ctx.fillStyle   = COLOR.vehicle;
  ctx.fill();
  ctx.strokeStyle = "#0d1117";
  ctx.lineWidth   = 1.5;
  ctx.stroke();

  // Heading arrow
  const rad    = (headingDeg - 90) * Math.PI / 180;
  const arrowLen = r + Math.max(4, cp * 0.25);
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x + Math.cos(rad) * arrowLen, y + Math.sin(rad) * arrowLen);
  ctx.strokeStyle = "#0d1117";
  ctx.lineWidth   = 2;
  ctx.stroke();
}
