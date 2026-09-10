import numpy as np

# Load the .npy file
data = np.load("./grid/obstacle_matrix.npy")

# View the contents, shape, and data type
print("Data:\n", data)
print("Shape:", data.shape)
print("Data Type:", data.dtype)
