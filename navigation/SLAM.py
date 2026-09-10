#SLAM.py marked.png marked.png marked.png marked.png --layout 2 2

import cv2
import numpy as np
import os
import matplotlib.pyplot as plt


class Slam:

    def __init__(
        self,
        image_path,
        output_dir="/outputs/grid",
        arena_width=2.0,
        arena_height=2.0,
        rows=20,
        cols=20,
        image_size=(2000, 2000),
        white_threshold=200,
        obstacle_percentage=0.15
    ):

        if rows <= 0 or cols <= 0:
            raise ValueError("rows and cols must be positive integers")

        if not 0 <= obstacle_percentage <= 1:
            raise ValueError("obstacle_percentage must be between 0 and 1")

        # ====================================================
        # VARIABLES
        # ====================================================

        self.image_path = image_path
        self.output_dir = output_dir

        self.arena_width = arena_width
        self.arena_height = arena_height

        self.rows = rows
        self.cols = cols
        self.tile_rows = 1
        self.tile_cols = 1

        self.image_size = image_size

        self.white_threshold = white_threshold
        self.obstacle_percentage = obstacle_percentage

        # Calculated later
        self.image = None
        self.grid_image = None

        self.cell_width = None
        self.cell_height = None

        self.obstacle_matrix = None
        self.grid_coordinates = []

        os.makedirs(self.output_dir, exist_ok=True)

    # ========================================================
    # 1. LOAD IMAGE
    # ========================================================

    def load_image(self):

        if isinstance(self.image_path, (str, os.PathLike)):
            image_paths = [[self.image_path]]
        else:
            image_paths = [list(row) for row in self.image_path]

            if not image_paths or not image_paths[0]:
                raise ValueError("image_path must contain at least one image")
            if any(len(row) != len(image_paths[0]) for row in image_paths):
                raise ValueError("all image rows must have the same number of images")

        self.tile_rows = len(image_paths)
        self.tile_cols = len(image_paths[0])

        image_rows = []
        for row_paths in image_paths:
            image_row = []
            for image_path in row_paths:
                image = cv2.imread(os.fspath(image_path))
                if image is None:
                    raise FileNotFoundError(f"Image not found: {image_path}")
                image_row.append(cv2.resize(image, self.image_size))
            image_rows.append(cv2.hconcat(image_row))

        self.image = cv2.vconcat(image_rows)
        self.rows *= self.tile_rows
        self.cols *= self.tile_cols

        height, width = self.image.shape[:2]

        self.cell_width = width // self.cols
        self.cell_height = height // self.rows

        print(f"Loaded {self.tile_rows} x {self.tile_cols} image tile(s)")
        print(f"Image size: {width} × {height}")
        print(f"Grid size: {self.rows} × {self.cols}")

    # ========================================================
    # 2. DETECT OBSTACLES
    # ========================================================

    def detect_obstacles(self):

        if self.image is None or self.cell_width is None or self.cell_height is None:
            raise RuntimeError("load_image() must be called before detect_obstacles()")

        self.obstacle_matrix = np.zeros(
            (self.rows, self.cols),
            dtype=np.uint8
        )

        for row in range(self.rows):

            for col in range(self.cols):

                x1 = col * self.cell_width
                y1 = row * self.cell_height

                x2 = (col + 1) * self.cell_width
                y2 = (row + 1) * self.cell_height

                cell = self.image[y1:y2, x1:x2]

                gray = cv2.cvtColor(
                    cell,
                    cv2.COLOR_BGR2GRAY
                )

                white_pixels = np.sum(
                    gray > self.white_threshold
                )

                total_pixels = gray.size

                white_percentage = (
                    white_pixels / total_pixels
                )

                if white_percentage > self.obstacle_percentage:

                    self.obstacle_matrix[row, col] = 1

        print("Obstacle detection completed")

    # ========================================================
    # 3. GENERATE COORDINATES
    # ========================================================

    def generate_coordinates(self):

        if self.cell_width is None or self.cell_height is None:
            raise RuntimeError("load_image() must be called before generate_coordinates()")
        if self.obstacle_matrix is None:
            raise RuntimeError("detect_obstacles() must be called before generate_coordinates()")

        self.grid_coordinates = []

        for row in range(self.rows):

            for col in range(self.cols):

                x1 = col * self.cell_width
                y1 = row * self.cell_height

                x2 = (col + 1) * self.cell_width
                y2 = (row + 1) * self.cell_height

                coordinate = (row + 1, col + 1)

                # Real-world position in meters
                real_x = (
                    col * self.arena_width / self.cols
                )

                real_y = (
                    row * self.arena_height / self.rows
                )

                self.grid_coordinates.append({

                    "coordinate": coordinate,

                    "pixel_bounds": (
                        x1, y1, x2, y2
                    ),

                    "real_position_m": (
                        real_x, real_y
                    ),

                    "obstacle": int(
                        self.obstacle_matrix[row, col]
                    )

                })

        print("Coordinates generated")

    # ========================================================
    # 4. DRAW GRID
    # ========================================================

    def draw_grid(self):

        if self.image is None or self.cell_width is None or self.cell_height is None:
            raise RuntimeError("load_image() must be called before draw_grid()")
        if self.obstacle_matrix is None:
            raise RuntimeError("detect_obstacles() must be called before draw_grid()")

        self.grid_image = self.image.copy()

        # Draw grid lines
        for i in range(1, self.rows):

            y = i * self.cell_height

            cv2.line(
                self.grid_image,
                (0, y),
                (self.image.shape[1], y),
                (0, 0, 255),
                2
            )

        for j in range(1, self.cols):

            x = j * self.cell_width

            cv2.line(
                self.grid_image,
                (x, 0),
                (x, self.image.shape[0]),
                (0, 0, 255),
                2
            )

        # Draw detected obstacle cells
        for row in range(self.rows):

            for col in range(self.cols):

                if self.obstacle_matrix[row, col] == 1:

                    x1 = col * self.cell_width
                    y1 = row * self.cell_height

                    x2 = (col + 1) * self.cell_width
                    y2 = (row + 1) * self.cell_height

                    cv2.rectangle(
                        self.grid_image,
                        (x1, y1),
                        (x2, y2),
                        (255, 0, 0),
                        3
                    )

        print("Grid drawn")

    # ========================================================
    # 5. SAVE GRID IMAGE
    # ========================================================

    def save_grid_image(self):

        if self.grid_image is None:
            raise RuntimeError("draw_grid() must be called before save_grid_image()")

        output_path = os.path.join(
            self.output_dir,
            "grid_image.png"
        )

        if not cv2.imwrite(output_path, self.grid_image):
            raise OSError(f"Could not save grid image: {output_path}")

        print(f"Grid image saved: {output_path}")

    # ========================================================
    # 6. SAVE OBSTACLE MATRIX
    # ========================================================

    def save_matrix(self):

        if self.obstacle_matrix is None:
            raise RuntimeError("detect_obstacles() must be called before save_matrix()")

        matrix_path = os.path.join(
            self.output_dir,
            "obstacle_matrix.npy"
        )

        np.save(
            matrix_path,
            self.obstacle_matrix
        )

        print(f"Matrix saved: {matrix_path}")

    # ========================================================
    # 7. DISPLAY OBSTACLE MATRIX
    # ========================================================

    def display_obstacle_matrix(self, show=False):

        if self.obstacle_matrix is None:
            raise RuntimeError(
                "detect_obstacles() must be called before "
                "display_obstacle_matrix()"
            )

        figure_width = max(8, self.cols * 0.35)
        figure_height = max(6, self.rows * 0.35)
        figure, axis = plt.subplots(figsize=(figure_width, figure_height))

        axis.imshow(
            self.obstacle_matrix,
            cmap="RdYlGn_r",
            vmin=0,
            vmax=1,
            interpolation="none"
        )
        axis.set_title("Obstacle Map")
        axis.set_xlabel("Column")
        axis.set_ylabel("Row")
        axis.set_xticks(np.arange(self.cols))
        axis.set_yticks(np.arange(self.rows))
        axis.set_xticklabels(np.arange(1, self.cols + 1))
        axis.set_yticklabels(np.arange(1, self.rows + 1))
        axis.set_xticks(np.arange(-0.5, self.cols, 1), minor=True)
        axis.set_yticks(np.arange(-0.5, self.rows, 1), minor=True)
        axis.grid(which="minor", color="black", linewidth=0.5)
        axis.tick_params(which="minor", bottom=False, left=False)

        for row in range(self.rows):
            for col in range(self.cols):
                axis.text(
                    col,
                    row,
                    str(int(self.obstacle_matrix[row, col])),
                    ha="center",
                    va="center",
                    color="white" if self.obstacle_matrix[row, col] else "black",
                    fontsize=7
                )

        figure.tight_layout()
        output_path = os.path.join(
            self.output_dir,
            "obstacle_matrix.png"
        )
        figure.savefig(output_path, dpi=200, bbox_inches="tight")
        print(f"Obstacle map saved: {output_path}")

        if show:
            plt.show()

        plt.close(figure)
        return figure, axis

    # ========================================================
    # 8. PRINT COORDINATES
    # ========================================================

    def print_coordinates(self):

        print("\nGRID COORDINATES\n")

        for item in self.grid_coordinates:

            print(
                f"{item['coordinate']} "
                f"-> Pixels: {item['pixel_bounds']} "
                f"-> Position: {item['real_position_m']} m "
                f"-> Obstacle: {item['obstacle']}"
            )

    # ========================================================
    # 9. PRINT MATRIX
    # ========================================================

    def print_matrix(self):

        print("\nOBSTACLE MATRIX\n")

        print(self.obstacle_matrix)

    # ========================================================
    # 10. PROCESS EVERYTHING
    # ========================================================

    def process(self):

        self.load_image()

        self.detect_obstacles()

        self.generate_coordinates()

        self.draw_grid()

        self.save_grid_image()

        self.save_matrix()

        self.display_obstacle_matrix()

        self.print_coordinates()

        self.print_matrix()

        return self.grid_image, self.obstacle_matrix


# ============================================================
# MAIN PROGRAM
# ============================================================

if __name__ == "__main__":

    # ===================== USER SETTINGS =====================
    # Arrange image paths row by row. This example creates a 2 x 2 map:
    #
    #   marked.png | marked.png
    #   marked.png | marked.png
    #
    # Replace these names with your image filenames.
    base_dir = os.path.dirname(os.path.abspath(__file__))
    image_names = [
        ["marked.png", "marked.png"],
        ["marked.png", "marked.png"]
    ]
    image_path = [
        [os.path.join(base_dir, image_name) for image_name in row]
        for row in image_names
    ]

    output_dir = os.path.abspath(
        os.path.join(base_dir, "..", "outputs", "grid")
    )
    show_matrix = True

    # Grid cells per image tile.
    rows_per_image = 20
    cols_per_image = 20

    mapper = Slam(

        image_path=image_path,

        output_dir=output_dir,

        arena_width=2.0,
        arena_height=2.0,

        rows=rows_per_image,
        cols=cols_per_image,

        image_size=(2000, 2000),

        white_threshold=200,

        obstacle_percentage=0.15
    )

    grid_image, obstacle_matrix = mapper.process()

    if show_matrix:
        mapper.display_obstacle_matrix(show=True)