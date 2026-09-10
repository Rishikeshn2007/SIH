"""
Route_planner.py
================
BFS-based route planner for the UGV navigation project.

Integrates with the existing Slam (SLAM.py / GridMapper) pipeline that
produces:
    outputs/grid/
    ├── grid_image.png
    ├── obstacle_matrix.png
    └── obstacle_matrix.npy          <- primary input for this module

Coordinate convention (used throughout this module)
----------------------------------------------------
All grid positions are expressed as **(row, col)** tuples using
**0-based indexing**:
    - row 0 is the TOP row of the matrix (first array axis)
    - col 0 is the LEFT-MOST column (second array axis)
    - (row, col) maps to matrix[row][col]

This matches NumPy's default array layout and the way Slam builds
obstacle_matrix.

Real-world relationship (20 x 20 grid, 2 m x 2 m arena):
    real_x_m = col  * (arena_width  / cols)   # metres from left edge
    real_y_m = row  * (arena_height / rows)   # metres from top edge
"""

from __future__ import annotations

import os
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Union

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Result container
# ---------------------------------------------------------------------------

@dataclass
class PlanResult:
    """
    Immutable container returned by RoutePlanner.find_route().

    Attributes
    ----------
    found : bool
        True if BFS reached the goal cell.
    path : list of (row, col) tuples
        Ordered sequence of grid cells from start to goal (inclusive).
        Empty list when no route exists.
    path_length : int
        Number of cells in the path (0 when no route found).
    cells_visited : int
        Total cells dequeued during BFS (useful for debugging and comparing
        algorithms).
    start : tuple[int, int]
        Start cell used for this search.
    goal : tuple[int, int]
        Goal cell used for this search.
    message : str
        Human-readable summary or error description.
    """

    found: bool
    path: List[Tuple[int, int]]
    path_length: int
    cells_visited: int
    start: Tuple[int, int]
    goal: Tuple[int, int]
    message: str = ""

    def path_as_real_world(
        self,
        arena_width: float = 2.0,
        arena_height: float = 2.0,
        rows: int = 20,
        cols: int = 20,
    ) -> List[Tuple[float, float]]:
        """
        Convert the grid path to real-world (x_m, y_m) coordinates.

        Uses the same mapping as Slam.generate_coordinates().

        Parameters
        ----------
        arena_width, arena_height : float
            Physical arena dimensions in metres.
        rows, cols : int
            Grid dimensions.

        Returns
        -------
        list of (x_m, y_m) tuples (origin at top-left corner of arena).
        """
        return [
            (
                col * arena_width  / cols,
                row * arena_height / rows,
            )
            for row, col in self.path
        ]

    def __repr__(self) -> str:
        status = "FOUND" if self.found else "NOT FOUND"
        return (
            f"PlanResult({status} | "
            f"start={self.start} goal={self.goal} | "
            f"length={self.path_length} cells | "
            f"visited={self.cells_visited})"
        )


# ---------------------------------------------------------------------------
# Main class
# ---------------------------------------------------------------------------

class RoutePlanner:
    """
    BFS-based shortest-path planner for a 2-D occupancy grid.

    The planner is designed around the output of the Slam class
    (SLAM.py) but accepts any conforming NumPy array or .npy file.

    Parameters
    ----------
    obstacle_matrix : np.ndarray or str or os.PathLike
        Either a 2-D NumPy array (dtype uint8, values 0/1) or a path to
        an obstacle_matrix.npy file produced by Slam.save_matrix().
    output_dir : str
        Directory where visualisation output is written.  Defaults to the
        outputs/grid/ directory relative to this file's location so that
        it matches the Slam pipeline layout.
    arena_width : float
        Physical arena width in metres.  Used only for real-world coordinate
        conversion, not for path finding.
    arena_height : float
        Physical arena height in metres.  Used only for real-world coordinate
        conversion, not for path finding.
    vehicle_length_m : float
        UGV body length in metres.  Stored for future obstacle-inflation use.
    vehicle_width_m : float
        UGV body width in metres.  Stored for future obstacle-inflation use.

    Notes
    -----
    Obstacle inflation / footprint expansion is NOT implemented yet.
    The method is_cell_valid() is the single authoritative choke-point for
    cell legality; override or extend it there when you add inflation.

    Example
    -------
    >>> import numpy as np
    >>> from Route_planner import RoutePlanner
    >>>
    >>> matrix = np.load("outputs/grid/obstacle_matrix.npy")
    >>> planner = RoutePlanner(matrix)
    >>>
    >>> planner.set_start(0, 0)      # top-left cell
    >>> planner.set_goal(19, 19)     # bottom-right cell
    >>>
    >>> result = planner.find_route()
    >>> print(result)
    >>> planner.visualize_route(result)
    """

    # 4-directional movement vectors: (delta_row, delta_col)
    _DIRECTIONS: Tuple[Tuple[int, int], ...] = (
        (-1,  0),   # up
        ( 1,  0),   # down
        ( 0, -1),   # left
        ( 0,  1),   # right
    )

    def __init__(
        self,
        obstacle_matrix: Union[np.ndarray, str, os.PathLike],
        output_dir: Optional[str] = None,
        arena_width: float = 2.0,
        arena_height: float = 2.0,
        vehicle_length_m: float = 0.35,
        vehicle_width_m: float = 0.20,
    ) -> None:

        # -- output directory --------------------------------------------------
        if output_dir is None:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            output_dir = os.path.abspath(
                os.path.join(base_dir, "..", "outputs", "grid")
            )
        self.output_dir: str = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

        # -- physical properties (stored for future use) -----------------------
        self.arena_width: float  = arena_width
        self.arena_height: float = arena_height
        self.vehicle_length_m: float = vehicle_length_m
        self.vehicle_width_m: float  = vehicle_width_m

        # -- internal state ----------------------------------------------------
        self._matrix: np.ndarray = self._load_matrix(obstacle_matrix)
        self.rows: int = self._matrix.shape[0]
        self.cols: int = self._matrix.shape[1]

        # Cell size in metres
        self.cell_width_m: float  = arena_width  / self.cols
        self.cell_height_m: float = arena_height / self.rows

        self._start: Optional[Tuple[int, int]] = None
        self._goal:  Optional[Tuple[int, int]] = None

        print(
            f"[RoutePlanner] Grid loaded: {self.rows}x{self.cols}  "
            f"| Cell size: {self.cell_width_m*100:.1f}x{self.cell_height_m*100:.1f} cm  "
            f"| Obstacles: {int(self._matrix.sum())}"
        )

    # ==========================================================================
    # 1.  MATRIX LOADING
    # ==========================================================================

    def _load_matrix(
        self,
        source: Union[np.ndarray, str, os.PathLike],
    ) -> np.ndarray:
        """
        Load and validate the obstacle matrix.

        Parameters
        ----------
        source : np.ndarray or path-like
            A ready-made NumPy array or a path to an .npy file.

        Returns
        -------
        np.ndarray
            A validated 2-D uint8 array containing only 0 (free) and 1
            (obstacle) values.

        Raises
        ------
        FileNotFoundError
            If source is a path that does not exist.
        ValueError
            If the loaded array is not 2-D or contains values other than 0/1.
        TypeError
            If source is neither an ndarray nor a path-like object.
        """
        if isinstance(source, np.ndarray):
            matrix = source.copy()
        elif isinstance(source, (str, os.PathLike)):
            path = os.fspath(source)
            if not os.path.isfile(path):
                raise FileNotFoundError(
                    f"[RoutePlanner] Obstacle matrix file not found: {path}"
                )
            matrix = np.load(path)
            print(f"[RoutePlanner] Loaded matrix from: {path}")
        else:
            raise TypeError(
                "[RoutePlanner] obstacle_matrix must be a NumPy array "
                "or a path to an .npy file."
            )

        # -- validation --------------------------------------------------------
        if matrix.ndim != 2:
            raise ValueError(
                f"[RoutePlanner] Expected a 2-D matrix, got shape {matrix.shape}."
            )

        unique_vals = set(np.unique(matrix).tolist())
        if not unique_vals.issubset({0, 1}):
            raise ValueError(
                f"[RoutePlanner] Matrix must contain only 0 (free) and 1 (obstacle). "
                f"Found: {unique_vals}"
            )

        return matrix.astype(np.uint8)

    @property
    def matrix(self) -> np.ndarray:
        """Read-only view of the obstacle matrix (0 = free, 1 = obstacle)."""
        return self._matrix.view()

    # ==========================================================================
    # 2.  START / GOAL SETTERS
    # ==========================================================================

    def set_start(self, row: int, col: int) -> None:
        """
        Set the start cell for path planning.

        Parameters
        ----------
        row : int
            0-based row index (0 = top row).
        col : int
            0-based column index (0 = left-most column).

        Raises
        ------
        ValueError
            If the cell is outside the grid or is an obstacle.
        """
        self._validate_endpoint(row, col, label="Start")
        self._start = (row, col)
        print(f"[RoutePlanner] Start set -> ({row}, {col})")

    def set_goal(self, row: int, col: int) -> None:
        """
        Set the goal cell for path planning.

        Parameters
        ----------
        row : int
            0-based row index (0 = top row).
        col : int
            0-based column index (0 = left-most column).

        Raises
        ------
        ValueError
            If the cell is outside the grid or is an obstacle.
        """
        self._validate_endpoint(row, col, label="Goal")
        self._goal = (row, col)
        print(f"[RoutePlanner] Goal  set -> ({row}, {col})")

    def _validate_endpoint(self, row: int, col: int, label: str) -> None:
        """Shared guard for set_start / set_goal."""
        if not (0 <= row < self.rows and 0 <= col < self.cols):
            raise ValueError(
                f"[RoutePlanner] {label} ({row}, {col}) is outside the "
                f"{self.rows}x{self.cols} grid."
            )
        if self._matrix[row, col] == 1:
            raise ValueError(
                f"[RoutePlanner] {label} ({row}, {col}) is inside an obstacle cell."
            )

    # ==========================================================================
    # 3.  CELL VALIDITY CHECK
    # ==========================================================================

    def is_cell_valid(self, row: int, col: int) -> bool:
        """
        Return True if a cell can be visited by the planner.

        This is the SINGLE AUTHORITATIVE CHOKE-POINT for cell legality.
        Future obstacle inflation or vehicle-footprint collision checking
        should be added here (or by subclassing and overriding this method).

        Current checks
        --------------
        1. Cell is inside the grid boundaries.
        2. Cell value is 0 (free).

        Planned future checks (not yet implemented)
        -------------------------------------------
        3. Inflate obstacles by ceil(vehicle_width / 2 / cell_size) cells
           so the vehicle body never overlaps a wall.
        4. Optionally check a rectangular footprint rotated to the heading
           direction for kinematic feasibility.

        Parameters
        ----------
        row, col : int
            0-based grid indices.

        Returns
        -------
        bool
        """
        if not (0 <= row < self.rows and 0 <= col < self.cols):
            return False
        return bool(self._matrix[row, col] == 0)

    # ==========================================================================
    # 4.  BFS ROUTE FINDING
    # ==========================================================================

    def find_route(self) -> PlanResult:
        """
        Run BFS from start to goal on the raw obstacle matrix.

        Algorithm
        ---------
        Standard FIFO-queue BFS on a 4-connected grid.  Each cell is visited
        at most once, so the first time the goal is reached it is guaranteed
        to be via the shortest path (minimum number of cell hops).

        Complexity
        ----------
        Time:  O(rows x cols)
        Space: O(rows x cols)  -- for the visited set and parent map

        Returns
        -------
        PlanResult
            Contains the path, statistics, and a found flag.
            If no route exists, path is empty and found is False.

        Raises
        ------
        RuntimeError
            If start or goal has not been set.
        """
        if self._start is None or self._goal is None:
            raise RuntimeError(
                "[RoutePlanner] Call set_start() and set_goal() before find_route()."
            )

        start = self._start
        goal  = self._goal

        # Edge case: start == goal
        if start == goal:
            return PlanResult(
                found=True,
                path=[start],
                path_length=1,
                cells_visited=1,
                start=start,
                goal=goal,
                message="Start and goal are the same cell.",
            )

        # -- BFS ---------------------------------------------------------------
        queue: deque = deque()
        queue.append(start)

        # parent[cell] = predecessor cell (enables path reconstruction)
        parent: dict = {start: None}

        cells_visited: int = 0

        while queue:
            current = queue.popleft()
            cells_visited += 1

            if current == goal:
                path = self._reconstruct_path(parent, goal)
                return PlanResult(
                    found=True,
                    path=path,
                    path_length=len(path),
                    cells_visited=cells_visited,
                    start=start,
                    goal=goal,
                    message=(
                        f"Route found: {len(path)} cells, "
                        f"{cells_visited} cells visited."
                    ),
                )

            for d_row, d_col in self._DIRECTIONS:
                neighbour = (current[0] + d_row, current[1] + d_col)
                if neighbour not in parent and self.is_cell_valid(*neighbour):
                    parent[neighbour] = current
                    queue.append(neighbour)

        # Queue exhausted without reaching goal
        return PlanResult(
            found=False,
            path=[],
            path_length=0,
            cells_visited=cells_visited,
            start=start,
            goal=goal,
            message=(
                f"No route found from {start} to {goal}. "
                f"Searched {cells_visited} cells."
            ),
        )

    # ==========================================================================
    # 5.  PATH RECONSTRUCTION
    # ==========================================================================

    @staticmethod
    def _reconstruct_path(
        parent: dict,
        goal: Tuple[int, int],
    ) -> List[Tuple[int, int]]:
        """
        Walk the parent map backwards from goal to start.

        Parameters
        ----------
        parent : dict
            Maps each visited cell to the cell from which it was first
            reached (None for the start cell).
        goal : tuple
            The destination cell.

        Returns
        -------
        list of (row, col) tuples
            Path ordered from start to goal (inclusive, both ends).
        """
        path: List[Tuple[int, int]] = []
        cell = goal
        while cell is not None:
            path.append(cell)
            cell = parent[cell]
        path.reverse()
        return path

    # ==========================================================================
    # 6.  VISUALISATION  (kept completely separate from BFS logic)
    # ==========================================================================

    def visualize_route(
        self,
        result: PlanResult,
        filename: str = "route_plan.png",
        show: bool = False,
    ) -> str:
        """
        Overlay the planned route on the grid image and save it.

        The base image (grid_image.png) produced by Slam is used if it exists
        in output_dir; otherwise a blank canvas is generated from the
        obstacle matrix.

        Visual encoding
        ---------------
        - Green filled circle  -- start cell
        - Red filled circle    -- goal cell
        - Cyan/yellow line     -- route (cell centre to cell centre)
        - Yellow filled dot    -- each waypoint along the route
        - Blue rectangle       -- obstacle cells (when drawing from scratch)

        Parameters
        ----------
        result : PlanResult
            The object returned by find_route().
        filename : str
            Output filename (written to output_dir).
        show : bool
            If True, display the image in an OpenCV window (requires a
            display; not suitable for headless servers).

        Returns
        -------
        str
            Absolute path to the saved image.

        Raises
        ------
        ValueError
            If result is not a PlanResult instance.
        """
        if not isinstance(result, PlanResult):
            raise ValueError(
                "[RoutePlanner] visualize_route() requires a PlanResult object "
                "returned by find_route()."
            )

        canvas = self._build_canvas()
        h, w = canvas.shape[:2]

        cell_px_w = w / self.cols
        cell_px_h = h / self.rows

        def cell_centre(row: int, col: int) -> Tuple[int, int]:
            """Pixel coordinate of the centre of a grid cell."""
            x = int((col + 0.5) * cell_px_w)
            y = int((row + 0.5) * cell_px_h)
            return x, y

        # -- draw path ---------------------------------------------------------
        if result.found and len(result.path) > 1:
            pts = [cell_centre(r, c) for r, c in result.path]

            # Thick route line
            for i in range(len(pts) - 1):
                cv2.line(canvas, pts[i], pts[i + 1], (255, 220, 0), 4)

            # Yellow waypoint dots
            dot_radius = max(4, int(min(cell_px_w, cell_px_h) * 0.18))
            for pt in pts[1:-1]:    # skip start & goal (drawn separately)
                cv2.circle(canvas, pt, dot_radius, (0, 255, 255), -1)

        # -- draw start --------------------------------------------------------
        start_pt = cell_centre(*result.start)
        marker_r = max(8, int(min(cell_px_w, cell_px_h) * 0.35))
        cv2.circle(canvas, start_pt, marker_r, (0, 200, 0), -1)          # green fill
        cv2.circle(canvas, start_pt, marker_r, (255, 255, 255), 2)       # white border
        cv2.putText(
            canvas, "S",
            (start_pt[0] - marker_r // 2, start_pt[1] + marker_r // 3),
            cv2.FONT_HERSHEY_SIMPLEX, marker_r / 30, (255, 255, 255), 2,
        )

        # -- draw goal ---------------------------------------------------------
        goal_pt = cell_centre(*result.goal)
        cv2.circle(canvas, goal_pt, marker_r, (0, 0, 200), -1)           # red fill
        cv2.circle(canvas, goal_pt, marker_r, (255, 255, 255), 2)        # white border
        cv2.putText(
            canvas, "G",
            (goal_pt[0] - marker_r // 2, goal_pt[1] + marker_r // 3),
            cv2.FONT_HERSHEY_SIMPLEX, marker_r / 30, (255, 255, 255), 2,
        )

        # -- status banner -----------------------------------------------------
        status_text = (
            f"Route: {result.path_length} cells | Visited: {result.cells_visited}"
            if result.found
            else "NO ROUTE FOUND"
        )
        banner_colour = (30, 160, 30) if result.found else (0, 0, 180)
        cv2.rectangle(canvas, (0, 0), (w, 50), banner_colour, -1)
        cv2.putText(
            canvas, status_text, (12, 34),
            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2,
        )

        # -- save --------------------------------------------------------------
        output_path = os.path.join(self.output_dir, filename)
        if not cv2.imwrite(output_path, canvas):
            raise OSError(
                f"[RoutePlanner] Could not save visualisation: {output_path}"
            )
        print(f"[RoutePlanner] Visualisation saved: {output_path}")

        if show:
            cv2.imshow("Route Plan", canvas)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return output_path

    # ==========================================================================
    # 7.  CANVAS HELPER
    # ==========================================================================

    def _build_canvas(self) -> np.ndarray:
        """
        Return an OpenCV BGR image to draw the route on.

        Tries to load grid_image.png from output_dir (the image produced by
        Slam.save_grid_image()).  Falls back to a synthetic canvas built
        directly from the obstacle matrix when that file is absent.

        Returns
        -------
        np.ndarray
            BGR image (H x W x 3, uint8).
        """
        grid_image_path = os.path.join(self.output_dir, "grid_image.png")

        if os.path.isfile(grid_image_path):
            canvas = cv2.imread(grid_image_path)
            if canvas is not None:
                return canvas
            print(
                "[RoutePlanner] Warning: grid_image.png exists but could not be "
                "read by OpenCV. Generating synthetic canvas."
            )

        # -- synthetic fallback ------------------------------------------------
        cell_px = 40                     # pixels per cell in fallback canvas
        h = self.rows * cell_px
        w = self.cols * cell_px
        canvas = np.full((h, w, 3), 30, dtype=np.uint8)  # dark background

        for r in range(self.rows):
            for c in range(self.cols):
                x1, y1 = c * cell_px, r * cell_px
                x2, y2 = x1 + cell_px, y1 + cell_px
                colour = (60, 60, 180) if self._matrix[r, c] == 1 else (50, 50, 50)
                cv2.rectangle(canvas, (x1, y1), (x2 - 1, y2 - 1), colour, -1)
                cv2.rectangle(canvas, (x1, y1), (x2, y2), (100, 100, 100), 1)

        print("[RoutePlanner] Synthetic canvas generated (grid_image.png not found).")
        return canvas

    # ==========================================================================
    # 8.  UTILITY / DEBUG
    # ==========================================================================

    def print_matrix(self) -> None:
        """Pretty-print the obstacle matrix to stdout."""
        print("\n[RoutePlanner] Obstacle matrix (0=free, 1=obstacle):\n")
        for r in range(self.rows):
            row_str = " ".join(str(int(v)) for v in self._matrix[r])
            print(f"  row {r:02d}: {row_str}")
        print()

    def __repr__(self) -> str:
        start = self._start if self._start else "not set"
        goal  = self._goal  if self._goal  else "not set"
        return (
            f"RoutePlanner("
            f"grid={self.rows}x{self.cols}, "
            f"start={start}, goal={goal}, "
            f"output_dir='{self.output_dir}')"
        )


# ==============================================================================
# USAGE EXAMPLE
# (Runs only when this file is executed directly, not when imported)
# ==============================================================================

if __name__ == "__main__":
    import sys

    # -- locate the obstacle matrix --------------------------------------------
    base_dir    = os.path.dirname(os.path.abspath(__file__))
    matrix_path = os.path.abspath(
        os.path.join(base_dir, "..", "outputs", "grid", "obstacle_matrix.npy")
    )

    if not os.path.isfile(matrix_path):
        print(
            f"[Example] '{matrix_path}' not found.\n"
            "Run SLAM.py first to generate the obstacle matrix, "
            "or supply a synthetic one."
        )
        # -- synthetic 20x20 matrix for self-contained testing -----------------
        rng = np.random.default_rng(42)
        synthetic = np.zeros((20, 20), dtype=np.uint8)
        obstacle_cells = rng.choice(400, size=40, replace=False)
        for idx in obstacle_cells:
            synthetic[idx // 20, idx % 20] = 1
        # Ensure corners are free
        for corner in [(0, 0), (0, 19), (19, 0), (19, 19)]:
            synthetic[corner] = 0
        matrix_source = synthetic
        print("[Example] Using synthetic 20x20 matrix with ~10 % random obstacles.")
    else:
        matrix_source = matrix_path

    # -- instantiate planner ---------------------------------------------------
    planner = RoutePlanner(
        obstacle_matrix=matrix_source,
        arena_width=2.0,
        arena_height=2.0,
        vehicle_length_m=0.35,
        vehicle_width_m=0.20,
    )

    print(planner)
    planner.print_matrix()

    # -- set endpoints ---------------------------------------------------------
    # Coordinates are 0-based (row, col): (0, 0) = top-left cell.
    planner.set_start(0, 0)
    planner.set_goal(37,37)

    # -- run BFS ---------------------------------------------------------------
    result = planner.find_route()
    print(result)

    if result.found:
        print(f"\n[Example] Path ({result.path_length} waypoints):")
        rw = result.path_as_real_world(rows=planner.rows, cols=planner.cols)
        for step, (r, c) in enumerate(result.path):
            print(
                f"  Step {step:>3}: grid ({r:>2}, {c:>2})"
                f"  ->  real ({rw[step][0]:.2f} m, {rw[step][1]:.2f} m)"
            )
    else:
        print(f"\n[Example] {result.message}")

    # -- save visualisation ----------------------------------------------------
    saved_path = planner.visualize_route(result, filename="route_plan.png")
    print(f"\n[Example] Route image saved to: {saved_path}")
    sys.exit(0 if result.found else 1)
