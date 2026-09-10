"""
generate_artificial_grid.py
===========================
Generates an artificial 20x5 obstacle matrix for testing.

Arena Specifications:
- Grid: 20 rows x 5 columns
- Real floor cell size: 30 cm x 30 cm (0.30 m x 0.30 m)
- Total arena dimensions: Length = 20 * 0.30 m = 6.0 m, Width = 5 * 0.30 m = 1.5 m
- UGV dimensions: 26 cm length x 12.5 cm width (0.26 m x 0.125 m)
- Free cell = 0, Obstacle cell = 1

Outputs saved to:
- outputs/grid/obstacle_matrix.npy  (and obstacle_matrix_20x5.npy)
- outputs/grid/obstacle_matrix_20x5.png
"""

import os
import numpy as np

# Try importing matplotlib for visual inspection
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


def create_20x5_obstacle_matrix() -> np.ndarray:
    """
    Creates a 20-row by 5-column arena with strategic obstacles,
    ensuring a valid collision-free path exists from (0, 0) or (0, 2)
    down to (19, 4) or (19, 2).
    """
    rows = 20
    cols = 5
    grid = np.zeros((rows, cols), dtype=np.uint8)

    # Place realistic obstacles (slalom / hallway pillars / barricades)
    # Row 2: Obstacle on column 1, 2
    grid[2, 1] = 1
    grid[2, 2] = 1

    # Row 5: Obstacle on column 3, 4
    grid[5, 3] = 1
    grid[5, 4] = 1

    # Row 8: Obstacle on column 0, 1
    grid[8, 0] = 1
    grid[8, 1] = 1

    # Row 11: Center obstacle on column 2
    grid[11, 2] = 1

    # Row 14: Obstacle on column 0 and column 4 (funnel)
    grid[14, 0] = 1
    grid[14, 4] = 1

    # Row 17: Obstacle on column 1, 2
    grid[17, 1] = 1
    grid[17, 2] = 1

    return grid


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.abspath(os.path.join(base_dir, "..", "outputs", "grid"))
    os.makedirs(output_dir, exist_ok=True)

    matrix = create_20x5_obstacle_matrix()

    # File paths
    npy_path_named = os.path.join(output_dir, "obstacle_matrix_20x5.npy")
    npy_path_default = os.path.join(output_dir, "obstacle_matrix.npy")
    png_path = os.path.join(output_dir, "obstacle_matrix_20x5.png")

    # Save NumPy matrix
    np.save(npy_path_named, matrix)
    np.save(npy_path_default, matrix)  # Overwrite default so server.py loads it directly

    print("=" * 60)
    print("  Artificial Obstacle Matrix Generated")
    print(f"  Dimensions: {matrix.shape[0]} rows x {matrix.shape[1]} cols")
    print(f"  Cell Size : 30 cm x 30 cm (0.30 m)")
    print(f"  Floor Size: {matrix.shape[0]*0.3:.1f} m (Length) x {matrix.shape[1]*0.3:.1f} m (Width)")
    print(f"  Car Size  : 26.0 cm (L) x 12.5 cm (W)")
    print(f"  Saved to  : {npy_path_default}")
    print(f"              {npy_path_named}")
    print("=" * 60)

    # Print ASCII representation
    print("\nGrid Map Preview (0 = Free, # = Obstacle):")
    header = "      " + " ".join([f"C{c}" for c in range(matrix.shape[1])])
    print(header)
    print("     " + "-" * (matrix.shape[1] * 3 + 1))
    for r in range(matrix.shape[0]):
        row_str = " ".join([" #" if matrix[r, c] == 1 else " ." for c in range(matrix.shape[1])])
        print(f"R{r:02d} | {row_str} |")
    print("     " + "-" * (matrix.shape[1] * 3 + 1))

    # Save PNG visualizer if matplotlib available
    if MATPLOTLIB_AVAILABLE:
        fig, ax = plt.subplots(figsize=(4, 10))
        # 0 = white (free), 1 = black (obstacle)
        cmap = plt.cm.colors.ListedColormap(["#ffffff", "#212529"])
        ax.imshow(matrix, cmap=cmap, origin="upper")

        # Gridlines
        ax.set_xticks(np.arange(-0.5, matrix.shape[1], 1), minor=True)
        ax.set_yticks(np.arange(-0.5, matrix.shape[0], 1), minor=True)
        ax.grid(which="minor", color="#0066cc", linestyle="-", linewidth=1)
        ax.tick_params(which="minor", size=0)

        ax.set_xticks(range(matrix.shape[1]))
        ax.set_yticks(range(matrix.shape[0]))
        ax.set_xlabel("Column (X)")
        ax.set_ylabel("Row (Y)")
        ax.set_title("20x5 Obstacle Matrix (30x30cm/cell)")
        plt.tight_layout()
        plt.savefig(png_path, dpi=200)
        plt.close()
        print(f"\nSaved PNG visualization to: {png_path}")


if __name__ == "__main__":
    main()
