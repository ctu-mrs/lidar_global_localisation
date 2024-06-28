
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

def compute_normals_angle_difference(pcd, grid=1.0):
    # Ensure normals are estimated
    if not pcd.has_normals():
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=100))
    
    # Extract points and normals
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)

    # Define grid bounds
    min_x, min_y = np.floor(np.min(points[:, :2], axis=0) / grid) * grid
    max_x, max_y = np.ceil(np.max(points[:, :2], axis=0) / grid) * grid

    # Calculate the size of the grid
    grid_width = int((max_x - min_x) / grid)
    grid_height = int((max_y - min_y) / grid)

    # Normalize and scale points to grid indices
    scaled_x = ((points[:, 0] - min_x) / grid).astype(int)
    scaled_y = ((points[:, 1] - min_y) / grid).astype(int)

    # Initialize sum of normals and count arrays
    normal_sums = np.zeros((grid_height, grid_width, 3))
    count = np.zeros((grid_height, grid_width))

    # Accumulate normals
    for (x, y), normal in zip(zip(scaled_x, scaled_y), normals):
        if 0 <= y < grid_height and 0 <= x < grid_width:
            normal_sums[y, x] += normal
            count[y, x] += 1

    # Compute average normals, avoiding division by zero
    valid_cells = count > 0
    average_normals = np.zeros_like(normal_sums)
    average_normals[valid_cells] = (normal_sums[valid_cells] / count[valid_cells][:, np.newaxis])
    average_normals /= np.linalg.norm(average_normals, axis=2, keepdims=True)
    average_normals[~valid_cells] = 0  # Reset invalid cells to zero

    # Calculate angle differences in degrees
    up_vector = np.array([0, 0, 1])
    angles = np.arccos(np.clip(np.einsum('ijk,k->ij', average_normals, up_vector), -1.0, 1.0))
    angle_degrees = np.degrees(angles)

    return angle_degrees

# Load the point cloud
#pcd = o3d.io.read_point_cloud("/home/michal/git/sprind_data/clouds/global/1719225305.811924468.pcd")
pcd = o3d.io.read_point_cloud("/home/michal/git/sprind_data/clouds/prior_clouds/5_2_2.pcd")
pcd = pcd.voxel_down_sample(voxel_size=0.1)
# Compute the angle differences
grid_size = 1.0  # You can adjust this value to change the resolution of pixels
angle_differences = compute_normals_angle_difference(pcd, grid=grid_size)

# Plot the angle differences
plt.figure(figsize=(8, 6))
plt.imshow(angle_differences, cmap='hot', origin='lower', interpolation='nearest')
plt.colorbar(label='Angle difference (degrees)')
plt.title(f'Angle Differences from Up Vector (Grid size: {grid_size})')
plt.show()
