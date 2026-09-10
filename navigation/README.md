# SLAM Image-to-Grid Mapper

`SLAM.py` converts one image or a grid of images into an obstacle map.
It detects obstacles, generates continuous grid coordinates, saves a grid image,
and saves the final obstacle matrix as a NumPy file.

## Requirements

Install the required packages in the project virtual environment:

```powershell
python -m pip install -r requirements.txt
```

The required packages are:

- `opencv-python`
- `numpy`
- `matplotlib`

## Configure the Images

Open `SLAM.py` and edit the `USER SETTINGS` section near the bottom.

For one image:

```python
image_names = [
    ["marked.png"]
]
```

For a 2 x 2 map, images are arranged row by row:

```python
image_names = [
    ["image11.png", "image12.png"],
    ["image21.png", "image22.png"]
]
```

The layout represents:

```text
image11 | image12
---------+--------
image21 | image22
```

The image files should be inside the `navigation` folder unless you provide a
relative or absolute path in `image_names`.

## Grid Settings

These values define the number of cells in each image tile:

```python
rows_per_image = 20
cols_per_image = 20
```

For a 2 x 2 image layout with these settings, the final obstacle matrix has
`40` rows and `40` columns. Coordinates continue across image boundaries from
the top-left image.

Other useful settings are:

```python
show_matrix = True

white_threshold = 200
obstacle_percentage = 0.15
```

`show_matrix = True` opens the Matplotlib window after processing. Set it to
`False` to save the files without opening a window.

## Run

From the `navigation` folder, run:

```powershell
python SLAM.py
```

The script uses the image names and settings defined inside `SLAM.py`.

## Output Files

Outputs are saved in:

```text
SIH/outputs/grid/
```

The files are:

```text
grid_image.png          # Input image or stitched multi-image map with grid lines
obstacle_matrix.png     # Graphical representation of the obstacle matrix
obstacle_matrix.npy     # Final obstacle matrix for Python use
```

Running the script again overwrites files with the same names in this folder.

## Load the Matrix

Use NumPy to load the final matrix:

```python
import numpy as np

matrix = np.load("../outputs/grid/obstacle_matrix.npy")
print(matrix)
print(matrix.shape)
```

A matrix value of `1` indicates an obstacle cell. A value of `0` indicates a
free cell.
