"""
Navigation_controller.py
========================
Manages all navigation state and command logic for the UGV.

This module sits between RoutePlanner and the Flask API layer.
It holds all mutable navigation state in one place and provides
clean methods that the Flask routes can call.

Architecture
------------
    RoutePlanner  →  NavigationController  →  Flask API  →  ESP32

Coordinate convention
---------------------
Internally, all positions use 0-based (row, col) tuples:
    row  : 0 = top    row of grid  (first  NumPy axis)
    col  : 0 = left-most column    (second NumPy axis)

ESP32 packets use {"x": col, "y": row}.
Translation is done at the API boundary (server.py), NOT here.

Heading convention
------------------
    0   = North  (decreasing row, i.e. moving up in the grid image)
    90  = East   (increasing col)
    180 = South  (increasing row)
    270 = West   (decreasing col)
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import sys

import numpy as np

# Ensure navigation/ directory is on the path so Route_planner is importable
# regardless of whether this file is run from the project root or navigation/.
_NAV_DIR = os.path.dirname(os.path.abspath(__file__))
if _NAV_DIR not in sys.path:
    sys.path.insert(0, _NAV_DIR)

from Route_planner import RoutePlanner, PlanResult

# ---------------------------------------------------------------------------
# Configurable constants — change here, not scattered through the code
# ---------------------------------------------------------------------------

CRUISE_SPEED: int   = 150    # PWM / speed units for straight movement (0-200)
TURN_SPEED: int     = 80     # PWM / speed units while turning
COMM_TIMEOUT_S: float = 5.0  # seconds before assuming vehicle is lost

# Heading values (degrees)
HEADING_NORTH: int = 0
HEADING_EAST:  int = 90
HEADING_SOUTH: int = 180
HEADING_WEST:  int = 270

# Direction vectors: heading → (delta_row, delta_col)
_HEADING_TO_DELTA: Dict[int, Tuple[int, int]] = {
    HEADING_NORTH: (-1,  0),
    HEADING_EAST:  ( 0,  1),
    HEADING_SOUTH: ( 1,  0),
    HEADING_WEST:  ( 0, -1),
}

# ---------------------------------------------------------------------------
# Stop command — returned whenever navigation is unsafe/invalid
# ---------------------------------------------------------------------------

_STOP_COMMAND: Dict = {
    "command":    "stop",
    "speed":      0,
    "turn_angle": None,
    "next_x":     None,
    "next_y":     None,
}


# ---------------------------------------------------------------------------
# Navigation state (single source of truth)
# ---------------------------------------------------------------------------

@dataclass
class NavigationState:
    """
    Complete snapshot of the navigation system at any point in time.

    All coordinate pairs are (row, col), 0-based.
    x / y equivalents: x = col, y = row.
    """

    # Planning
    source:      Optional[Tuple[int, int]] = None
    destination: Optional[Tuple[int, int]] = None
    route:       List[Tuple[int, int]]     = field(default_factory=list)

    # Vehicle
    current_position: Optional[Tuple[int, int]] = None
    current_heading:  int                        = HEADING_NORTH

    # Progress
    current_route_index: int  = 0
    running:             bool = False
    completed:           bool = False

    # Comms
    last_command:        Optional[Dict] = None
    last_packet_received: Optional[Dict] = None
    last_command_sent:   Optional[Dict] = None
    last_update_time:    float          = 0.0

    # Diagnostics
    error_message: str = ""

    # Grid dimensions (filled when navigation starts)
    grid_rows: int = 0
    grid_cols: int = 0

    def status_label(self) -> str:
        """Return a human-readable status string for the UI."""
        if self.error_message:
            return "ERROR"
        if self.completed:
            return "COMPLETED"
        if self.running:
            return "RUNNING"
        return "IDLE"


# ---------------------------------------------------------------------------
# NavigationController
# ---------------------------------------------------------------------------

class NavigationController:
    """
    Controls route following for the UGV.

    Usage
    -----
    ::

        controller = NavigationController(matrix_path="outputs/grid/obstacle_matrix.npy")

        result = controller.start(source=(0, 0), destination=(19, 19))

        # On each ESP32 heartbeat:
        command = controller.update_vehicle_state(x=col, y=row, heading=90, running=True)

        # Anytime:
        status = controller.get_status()

        controller.stop()

    Parameters
    ----------
    matrix_path : str or os.PathLike
        Path to the obstacle_matrix.npy file produced by Slam.
    arena_width, arena_height : float
        Physical arena dimensions in metres.
    vehicle_length_m, vehicle_width_m : float
        Vehicle dimensions (stored for future inflation use).
    """

    def __init__(
        self,
        matrix_path: Optional[str | os.PathLike] = None,
        arena_width: float = 1.5,
        arena_height: float = 6.0,
        vehicle_length_m: float = 0.26,
        vehicle_width_m: float  = 0.125,
    ) -> None:

        # Resolve default matrix path relative to this file
        if matrix_path is None:
            base = os.path.dirname(os.path.abspath(__file__))
            matrix_path = os.path.abspath(
                os.path.join(base, "..", "outputs", "grid", "obstacle_matrix.npy")
            )

        self.matrix_path: str = os.fspath(matrix_path)
        self.arena_width:       float = arena_width
        self.arena_height:      float = arena_height
        self.vehicle_length_m:  float = vehicle_length_m
        self.vehicle_width_m:   float = vehicle_width_m

        self._state: NavigationState = NavigationState()
        self._matrix: Optional[np.ndarray] = None

        # Pre-load the matrix so /map is always available even before planning
        self._try_load_matrix()

    # ======================================================================
    # Matrix loading
    # ======================================================================

    def _try_load_matrix(self) -> None:
        """Load the obstacle matrix from disk if the file exists."""
        if os.path.isfile(self.matrix_path):
            try:
                self._matrix = np.load(self.matrix_path)
                print(f"[NavController] Matrix loaded: {self._matrix.shape}")
            except Exception as exc:
                print(f"[NavController] Warning: could not load matrix — {exc}")
                self._matrix = None
        else:
            print(f"[NavController] Warning: matrix not found at {self.matrix_path}")

    def reload_matrix(self) -> None:
        """Reload the obstacle matrix from disk (call after SLAM runs again)."""
        self._try_load_matrix()

    @property
    def matrix(self) -> Optional[np.ndarray]:
        """The current obstacle matrix (may be None if not yet loaded)."""
        return self._matrix

    # ======================================================================
    # Navigation start / stop
    # ======================================================================

    def start(
        self,
        source: Tuple[int, int],
        destination: Tuple[int, int],
    ) -> Dict:
        """
        Plan a route and begin navigation.

        Parameters
        ----------
        source : (row, col)
            Starting grid cell (0-based).
        destination : (row, col)
            Target grid cell (0-based).

        Returns
        -------
        dict with keys: status, source, destination, route, route_length,
        grid_rows, grid_cols, message
        """
        # Reload matrix so we always use the latest SLAM output
        self._try_load_matrix()

        if self._matrix is None:
            return self._error_response(
                "Obstacle matrix not available. Run SLAM.py first."
            )

        # ── plan ──────────────────────────────────────────────────────────
        try:
            planner = RoutePlanner(
                obstacle_matrix=self._matrix,
                arena_width=self.arena_width,
                arena_height=self.arena_height,
                vehicle_length_m=self.vehicle_length_m,
                vehicle_width_m=self.vehicle_width_m,
            )
            planner.set_start(*source)
            planner.set_goal(*destination)
            result: PlanResult = planner.find_route()
        except ValueError as exc:
            return self._error_response(str(exc))

        if not result.found:
            return self._error_response(
                f"No collision-free route from {source} to {destination}."
            )

        # ── update state ──────────────────────────────────────────────────
        rows, cols = self._matrix.shape
        self._state = NavigationState(
            source=source,
            destination=destination,
            route=result.path,
            current_position=source,
            current_heading=HEADING_NORTH,
            current_route_index=0,
            running=True,
            completed=False,
            last_update_time=time.time(),
            grid_rows=rows,
            grid_cols=cols,
        )

        print(
            f"[NavController] Navigation started: {source} -> {destination}, "
            f"{result.path_length} waypoints."
        )

        return {
            "status":       "started",
            "source":       list(source),
            "destination":  list(destination),
            "route":        [list(wp) for wp in result.path],
            "route_length": result.path_length,
            "cells_visited": result.cells_visited,
            "grid_rows":    rows,
            "grid_cols":    cols,
            "message":      result.message,
        }

    def stop(self) -> Dict:
        """
        Halt navigation immediately.

        Returns
        -------
        dict with the stop command so the caller can forward it to the vehicle.
        """
        self._state.running   = False
        self._state.completed = False
        self._state.error_message = ""
        print("[NavController] Navigation stopped.")
        cmd = dict(_STOP_COMMAND)
        cmd["timestamp"] = time.time()
        self._state.last_command = cmd
        self._state.last_command_sent = cmd
        return {"status": "stopped", "command": cmd}

    # ======================================================================
    # Vehicle state update  (called on every POST /car/state)
    # ======================================================================

    def update_vehicle_state(
        self,
        x: int,
        y: int,
        heading: int,
        running: bool,
    ) -> Dict:
        """
        Process an incoming ESP32 heartbeat and return the next command.

        Parameters
        ----------
        x : int
            Vehicle column position (0-based).
        y : int
            Vehicle row position (0-based).
        heading : int
            Current vehicle heading in degrees (0/90/180/270).
        running : bool
            Whether the vehicle reports itself as running.

        Returns
        -------
        dict with keys: next_x, next_y, command, speed, turn_angle, timestamp
        """
        row, col = y, x     # translate ESP32 coords → internal (row, col)

        # Record raw packet
        packet = {
            "x": x, "y": y,
            "heading": heading,
            "running": running,
            "timestamp": time.time(),
        }
        self._state.last_packet_received = packet
        self._state.last_update_time     = time.time()

        # ── safety checks ─────────────────────────────────────────────────
        if not self._state.running:
            return self._build_stop_response("Navigation not active.")

        if self._state.completed:
            return self._build_stop_response("Destination already reached.")

        if not running:
            return self._build_stop_response("Vehicle reports it is not running.")

        if self._matrix is not None:
            rows, cols = self._matrix.shape
            if not (0 <= row < rows and 0 <= col < cols):
                return self._build_stop_response(
                    f"Received out-of-bounds position: ({row}, {col})."
                )

        # ── update state ──────────────────────────────────────────────────
        self._state.current_position = (row, col)
        self._state.current_heading  = heading

        # Advance the route index to reflect where the vehicle actually is
        self._advance_route_index((row, col))

        # ── compute command ───────────────────────────────────────────────
        cmd = self._compute_command()
        self._state.last_command      = cmd
        self._state.last_command_sent = cmd

        return cmd

    def check_timeout(self) -> Optional[Dict]:
        """
        Return a stop command dict if the vehicle has not reported recently.

        Returns None if comms are healthy or navigation is not running.
        """
        if not self._state.running:
            return None
        elapsed = time.time() - self._state.last_update_time
        if elapsed > COMM_TIMEOUT_S:
            self._state.running = False
            self._state.error_message = (
                f"Communication timeout after {elapsed:.1f}s."
            )
            print(f"[NavController] {self._state.error_message}")
            return self._build_stop_response(self._state.error_message)
        return None

    # ======================================================================
    # Route-index advancement
    # ======================================================================

    def _advance_route_index(self, current_pos: Tuple[int, int]) -> None:
        """
        Move current_route_index forward to the furthest waypoint the vehicle
        has already reached or passed (handles minor positioning noise).
        """
        route = self._state.route
        if not route:
            return

        # Find the furthest waypoint that matches the current cell
        for i in range(len(route) - 1, self._state.current_route_index - 1, -1):
            if route[i] == current_pos:
                self._state.current_route_index = i
                break

        # Check for destination arrival
        if (
            self._state.current_route_index >= len(route) - 1
            and current_pos == self._state.destination
        ):
            self._state.running   = False
            self._state.completed = True
            print("[NavController] Destination reached!")

    # ======================================================================
    # Command computation
    # ======================================================================

    def _compute_command(self) -> Dict:
        """
        Determine the movement command from current position + heading +
        next waypoint.

        Returns
        -------
        dict with: command, speed, turn_angle, next_x, next_y, timestamp
        """
        # Completed / no route
        if self._state.completed:
            return self._build_stop_response("Destination reached.")

        route = self._state.route
        if not route:
            return self._build_stop_response("No route loaded.")

        next_index = self._state.current_route_index + 1
        if next_index >= len(route):
            # Already at or past the last waypoint
            self._state.completed = True
            self._state.running   = False
            return self._build_stop_response("Destination reached.")

        next_wp = route[next_index]
        current = self._state.current_position

        required_hdg = self._required_heading(current, next_wp)
        current_hdg  = self._state.current_heading

        next_x = next_wp[1]  # col
        next_y = next_wp[0]  # row

        if required_hdg == current_hdg:
            # Aligned — drive straight
            cmd = {
                "command":    "move_forward",
                "speed":      CRUISE_SPEED,
                "turn_angle": None,
                "next_x":     next_x,
                "next_y":     next_y,
                "timestamp":  time.time(),
            }
        else:
            # Need to turn first
            turn_cmd, turn_angle = self._turn_direction(current_hdg, required_hdg)
            cmd = {
                "command":    turn_cmd,
                "speed":      TURN_SPEED,
                "turn_angle": turn_angle,
                "next_x":     next_x,
                "next_y":     next_y,
                "timestamp":  time.time(),
            }

        return cmd

    # ======================================================================
    # Heading helpers
    # ======================================================================

    @staticmethod
    def _required_heading(
        current: Tuple[int, int],
        nxt: Tuple[int, int],
    ) -> int:
        """
        Return the compass heading (0/90/180/270) that points from
        current to nxt.

        Parameters
        ----------
        current, nxt : (row, col)
        """
        dr = nxt[0] - current[0]
        dc = nxt[1] - current[1]

        if dr == -1 and dc == 0:
            return HEADING_NORTH
        if dr == 1  and dc == 0:
            return HEADING_SOUTH
        if dr == 0  and dc == 1:
            return HEADING_EAST
        if dr == 0  and dc == -1:
            return HEADING_WEST

        # Diagonal or same cell — should not happen with 4-connected BFS
        # Fall back to North as a safe default
        return HEADING_NORTH

    @staticmethod
    def _turn_direction(
        current_hdg: int,
        required_hdg: int,
    ) -> Tuple[str, int]:
        """
        Determine the shortest turn direction and angle.

        Returns
        -------
        (command_str, turn_angle_degrees)
        e.g. ("turn_right", 90) or ("turn_left", 90) or ("turn_right", 180)
        """
        diff = (required_hdg - current_hdg) % 360

        if diff == 90:
            return "turn_right", 90
        if diff == 270:
            return "turn_left", 90
        if diff == 180:
            # 180° turn — choose right by convention
            return "turn_right", 180

        # Same heading (diff == 0) — caller should not reach here, but be safe
        return "move_forward", 0

    # ======================================================================
    # Status / serialisation
    # ======================================================================

    def get_status(self) -> Dict:
        """
        Return the complete navigation state as a JSON-serialisable dict.
        Safe to call at any time.
        """
        s = self._state
        route_serialised = [list(wp) for wp in s.route]

        pos  = list(s.current_position) if s.current_position else None
        src  = list(s.source)           if s.source           else None
        dest = list(s.destination)      if s.destination      else None

        return {
            "status":              s.status_label(),
            "running":             s.running,
            "completed":           s.completed,
            "error":               s.error_message,

            # Positions (as [row, col])
            "source":              src,
            "destination":         dest,
            "current_position":    pos,

            # ESP32-friendly equivalents
            "current_x":           pos[1] if pos else None,
            "current_y":           pos[0] if pos else None,

            "current_heading":     s.current_heading,
            "current_route_index": s.current_route_index,

            "route":               route_serialised,
            "route_length":        len(s.route),

            "grid_rows":           s.grid_rows,
            "grid_cols":           s.grid_cols,

            "last_command":        s.last_command,
            "last_packet_received": s.last_packet_received,
            "last_command_sent":   s.last_command_sent,
            "last_update_time":    s.last_update_time,

            # Comms health
            "seconds_since_update": (
                round(time.time() - s.last_update_time, 2)
                if s.last_update_time else None
            ),
        }

    def get_matrix_as_list(self) -> Optional[List[List[int]]]:
        """Return the obstacle matrix as a plain Python list of lists."""
        if self._matrix is None:
            return None
        return self._matrix.tolist()

    # ======================================================================
    # Internal helpers
    # ======================================================================

    @staticmethod
    def _error_response(message: str) -> Dict:
        return {"status": "error", "message": message}

    def _build_stop_response(self, reason: str = "") -> Dict:
        cmd = dict(_STOP_COMMAND)
        cmd["timestamp"] = time.time()
        cmd["reason"]    = reason
        self._state.last_command      = cmd
        self._state.last_command_sent = cmd
        return cmd
