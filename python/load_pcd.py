import numpy as np
import matplotlib.pyplot as plt
from pypcd4 import PointCloud
import open3d as o3d
import cv2
def compute_2d_density_map_fixed(pcd, cell_size=1.0):
    """
    Computes a 2D density map from a 3D point cloud by projecting it onto the XY plane.
    The grid cell size is fixed, and the grid dimensions adjust based on the point cloud bounds.
    
    Parameters:
        pcd (np.array): A Nx3 numpy array of 3D points.
        cell_size (float): The size of each cell in the grid (defines resolution).
    
    Returns:
        None: Displays the density map as a heatmap.
    """
    # Project the point cloud onto the XY plane
    points_xy = pcd[:, :2]

    # Calculate the bounds needed to cover all points with the fixed cell size
    x_min, x_max = np.floor(np.min(points_xy[:, 0])), np.ceil(np.max(points_xy[:, 0]))
    y_min, y_max = np.floor(np.min(points_xy[:, 1])), np.ceil(np.max(points_xy[:, 1]))

    # Calculate the number of bins along each axis based on the cell size
    x_bins = int((x_max - x_min) / cell_size)
    y_bins = int((y_max - y_min) / cell_size)

    # Create a 2D histogram with the fixed cell size
    density, xedges, yedges = np.histogram2d(points_xy[:, 0], points_xy[:, 1], bins=[x_bins, y_bins], range=[[x_min, x_max], [y_min, y_max]])

    # Plotting the density map
    plt.figure(figsize=(8, 6))
    plt.imshow(density.T, extent=[x_min, x_max, y_min, y_max], origin='lower', cmap='hot', interpolation='nearest')
    plt.colorbar(label='Density')
    plt.title('2D Density Map with Fixed Cell Size')
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.show()

def create_height_map(pcd, cell_size=1.0):
    """
    Creates a 2D map from a 3D point cloud by projecting it onto the XY plane.
    Each cell in the resulting map stores the highest Z value of the points that fall into that cell.
    
    Parameters:
        pcd (np.array): A Nx3 numpy array of 3D points.
        cell_size (float): The size of each cell in the grid (defines resolution).
    
    Returns:
        None: Displays the height map as an image.
    """
    # Project the point cloud onto the XY plane and separate the Z coordinates
    points_xy = pcd[:, :2]
    z_values = pcd[:, 2]

    # Calculate the bounds needed to cover all points with the fixed cell size
    x_min, x_max = np.floor(np.min(points_xy[:, 0])), np.ceil(np.max(points_xy[:, 0]))
    y_min, y_max = np.floor(np.min(points_xy[:, 1])), np.ceil(np.max(points_xy[:, 1]))

    # Calculate the number of bins along each axis based on the cell size
    x_bins = int((x_max - x_min) / cell_size)
    y_bins = int((y_max - y_min) / cell_size)

    # Initialize the height map array with negative infinity (to store maximum values)
    height_map = np.full((y_bins, x_bins), -np.inf)

    # Assign each point to the appropriate cell and update the max Z value
    for x, y, z in zip(points_xy[:, 0], points_xy[:, 1], z_values):
        x_idx = int((x - x_min) / cell_size)
        y_idx = int((y - y_min) / cell_size)

        if(x_idx < 0 or x_idx >= x_bins or y_idx<0 or y_idx>=y_bins):
            continue
        if height_map[y_idx, x_idx] <= z:
            height_map[y_idx, x_idx] = z

    print("created heightmap", height_map.shape)
    return heightmap
    # Plotting the height map
    
    #plt.figure(figsize=(8, 6))
    #plt.imshow(height_map, extent=[x_min, x_max, y_min, y_max], origin='lower', cmap='terrain', interpolation='nearest')
    #plt.colorbar(label='Max Z value')
    #plt.title('Height Map with Fixed Cell Size')
    #plt.xlabel('X coordinate')
    #plt.ylabel('Y coordinate')
    #plt.show()



# Example usage
# Generate some synthetic point cloud data
#np.random.seed(0)
#synthetic_pcd = np.random.rand(1000, 3) * 100  # 1000 points in a 100x100x100 volume

#import ply
#pcd = o3d.io.read_point_cloud("/home/michal/git/LidarLocalisationProject/map_tiles_ply/5_2_3.ply")
#o3d.io.write_point_cloud("/home/michal/git/LidarLocalisationProject/map_tiles_ply/5_2_3.pcd", pcd)
pcd = o3d.io.read_point_cloud("/home/michal/git/sprind_data/clouds/global/1719225305.811924468.pcd")
downpcd = pcd.voxel_down_sample(voxel_size=0.5)
pc_arr = np.asarray(downpcd.points)

#o3d.visualization.draw_geometries([downpcd])

#pc = pcl.load("/home/michal/git/LidarLocalisationProject/map_tiles_ply/5_0_0.ply")

#pc = PointCloud.from_path("/home/michal/git/sprind_data/clouds/global/1719225305.811924468.pcd")
#pc = PointCloud.from_path("/home/michal/git/sprind_data/clouds/prior_clouds/5.pcd")
#pc_arr = pc.numpy(("x", "y", "z"))
#compute_2d_density_map_fixed(pc_arr, cell_size=1.0)



pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
o3d.visualization.draw_geometries([pcd], point_show_normal=True)

create_height_map(pc_arr, cell_size=1.0)
