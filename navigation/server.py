"""
server.py
=========
Lightweight Flask server for UGV navigation.

Run:
    cd navigation
    python server.py

Endpoints
---------
POST /navigation/start      — plan a route and begin navigation
POST /navigation/stop       — halt navigation
GET  /navigation/status     — full navigation state (polled by dashboard)
POST /car/state             — receive ESP32 heartbeat, return movement command
GET  /car/state             — last known vehicle state (debug)
GET  /route                 — planned route as JSON
GET  /map                   — obstacle matrix as 2-D JSON array
GET  /                      — serve the dashboard (index.html)

Coordinate convention at the API boundary
-----------------------------------------
    ESP32 / API: {"x": col, "y": row}   (x = column, y = row)
    Internal:    (row, col) tuples       (matches NumPy layout)

Translation happens only in this file, not in NavigationController.
"""

from __future__ import annotations

import os
import sys
import collections
import threading
import time

# ---------------------------------------------------------------------------
# Path setup — ensure the navigation/ directory is importable regardless of
# whether the server is started from project root or from navigation/
# ---------------------------------------------------------------------------
_NAV_DIR = os.path.dirname(os.path.abspath(__file__))
if _NAV_DIR not in sys.path:
    sys.path.insert(0, _NAV_DIR)

from flask import Flask, jsonify, render_template, request

from Navigation_controller import NavigationController

# ---------------------------------------------------------------------------
# App setup
# ---------------------------------------------------------------------------

BASE_DIR     = os.path.dirname(os.path.abspath(__file__))
MATRIX_PATH  = os.path.abspath(
    os.path.join(BASE_DIR, "..", "outputs", "grid", "obstacle_matrix_20x5.npy")
)

app = Flask(
    __name__,
    template_folder=os.path.join(BASE_DIR, "templates"),
    static_folder=os.path.join(BASE_DIR, "static"),
)

# Single shared controller instance (thread-safe via lock)
controller = NavigationController(matrix_path=MATRIX_PATH)
_lock = threading.Lock()

# Packet log: keep last 30 entries  ({"direction": "rx"/"tx", "data": {...}})
_packet_log: collections.deque = collections.deque(maxlen=30)

# ---------------------------------------------------------------------------
# Helper utilities
# ---------------------------------------------------------------------------

def _ok(data: dict, code: int = 200):
    """Wrap a success payload in the standard envelope."""
    return jsonify({"status": "ok", "data": data}), code


def _err(message: str, code: int = 400):
    """Wrap an error in the standard envelope."""
    return jsonify({"status": "error", "message": message}), code


def _log_rx(packet: dict) -> None:
    _packet_log.append({"direction": "rx", "data": packet, "ts": time.time()})


def _log_tx(packet: dict) -> None:
    _packet_log.append({"direction": "tx", "data": packet, "ts": time.time()})


# ---------------------------------------------------------------------------
# Dashboard
# ---------------------------------------------------------------------------

@app.route("/")
def index():
    """Serve the development dashboard."""
    return render_template("index.html")


# ---------------------------------------------------------------------------
# POST /navigation/start
# ---------------------------------------------------------------------------

@app.route("/navigation/start", methods=["POST"])
def navigation_start():
    """
    Start a navigation task.

    Request body (JSON)
    -------------------
    {
        "source":      [row, col],   e.g. [0, 0]
        "destination": [row, col]    e.g. [19, 19]
    }

    Returns the planned route and grid metadata.
    """
    body = request.get_json(silent=True) or {}

    src_raw  = body.get("source")
    dest_raw = body.get("destination")

    if src_raw is None or dest_raw is None:
        return _err('Request must contain "source" and "destination" arrays.')

    try:
        source      = (int(src_raw[0]),  int(src_raw[1]))
        destination = (int(dest_raw[0]), int(dest_raw[1]))
    except (TypeError, IndexError, ValueError):
        return _err(
            '"source" and "destination" must be [row, col] integer arrays.'
        )

    with _lock:
        result = controller.start(source=source, destination=destination)

    if result.get("status") == "error":
        return _err(result["message"], code=400)

    _log_tx(result)
    return _ok(result, 200)


# ---------------------------------------------------------------------------
# POST /navigation/stop
# ---------------------------------------------------------------------------

@app.route("/navigation/stop", methods=["POST"])
def navigation_stop():
    """Stop navigation immediately."""
    with _lock:
        result = controller.stop()
    _log_tx(result)
    return _ok(result)


# ---------------------------------------------------------------------------
# GET /navigation/status
# ---------------------------------------------------------------------------

@app.route("/navigation/status", methods=["GET"])
def navigation_status():
    """
    Return the full navigation state.
    Polled by the dashboard every ~750 ms.
    """
    with _lock:
        # Check comms timeout while we have the lock
        timeout_cmd = controller.check_timeout()
        status = controller.get_status()

    if timeout_cmd:
        _log_tx(timeout_cmd)

    # Include the packet log so the UI can show it
    status["packet_log"] = list(_packet_log)

    return _ok(status)


# ---------------------------------------------------------------------------
# POST /car/state  — primary ESP32 endpoint
# ---------------------------------------------------------------------------

@app.route("/car/state", methods=["POST"])
def car_state_post():
    """
    Receive the vehicle's current state and return a movement command.

    Request body (JSON)
    -------------------
    {
        "x":       col,      integer, 0-based column
        "y":       row,      integer, 0-based row
        "heading": degrees,  0=North 90=East 180=South 270=West
        "running": bool
    }

    Response (JSON)
    ---------------
    {
        "next_x":     col,
        "next_y":     row,
        "command":    "move_forward" | "turn_left" | "turn_right" |
                      "move_backward" | "stop",
        "speed":      0-200,
        "turn_angle": degrees | null
    }
    """
    body = request.get_json(silent=True) or {}

    # ── validate required fields ──────────────────────────────────────────
    required = ("x", "y", "heading", "running")
    missing  = [k for k in required if k not in body]
    if missing:
        return _err(f"Missing fields: {missing}")

    try:
        x       = int(body["x"])
        y       = int(body["y"])
        heading = int(body["heading"])
        running = bool(body["running"])
    except (TypeError, ValueError):
        return _err("x, y, heading must be integers; running must be boolean.")

    if heading not in (0, 90, 180, 270):
        return _err("heading must be 0, 90, 180, or 270.")

    _log_rx({"x": x, "y": y, "heading": heading, "running": running})

    with _lock:
        command = controller.update_vehicle_state(
            x=x, y=y, heading=heading, running=running
        )

    _log_tx(command)
    return jsonify(command), 200


# ---------------------------------------------------------------------------
# GET /car/state  — optional debug endpoint
# ---------------------------------------------------------------------------

@app.route("/car/state", methods=["GET"])
def car_state_get():
    """Return the last known vehicle state (for debugging)."""
    with _lock:
        status = controller.get_status()

    vehicle = {
        "x":              status.get("current_x"),
        "y":              status.get("current_y"),
        "heading":        status.get("current_heading"),
        "nav_status":     status.get("status"),
        "last_command":   status.get("last_command"),
    }
    return _ok(vehicle)


# ---------------------------------------------------------------------------
# GET /route
# ---------------------------------------------------------------------------

@app.route("/route", methods=["GET"])
def get_route():
    """Return the current planned route as a list of [row, col] pairs."""
    with _lock:
        status = controller.get_status()

    route = status.get("route", [])
    return _ok({
        "route":        route,
        "route_length": len(route),
        "source":       status.get("source"),
        "destination":  status.get("destination"),
    })


# ---------------------------------------------------------------------------
# GET /map
# ---------------------------------------------------------------------------

@app.route("/map", methods=["GET"])
def get_map():
    """
    Return the obstacle matrix as a 2-D array.

    Response: {"rows": N, "cols": M, "matrix": [[0,1,...], ...]}
    0 = free cell, 1 = obstacle cell.
    """
    with _lock:
        matrix = controller.get_matrix_as_list()

    if matrix is None:
        return _err(
            "Obstacle matrix not loaded. Run SLAM.py to generate it.",
            code=404,
        )

    rows = len(matrix)
    cols = len(matrix[0]) if rows else 0
    return _ok({"rows": rows, "cols": cols, "matrix": matrix})


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print("=" * 60)
    print("  UGV Navigation Server")
    print(f"  Matrix: {MATRIX_PATH}")
    print("  Dashboard: http://localhost:5000")
    print("=" * 60)
    app.run(host="0.0.0.0", port=5000, debug=False)
