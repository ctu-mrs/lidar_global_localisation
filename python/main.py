import numpy as np
import matplotlib.pyplot as plt
from pypcd4 import PointCloud
import open3d as o3d
import cv2

from data_loader import DataLoader
from cloud_processing import CloudProcessing


voxel_size = 0.5
pixel_size = 1.0

#load pointclouds
loader = DataLoader()
pcd_file = "/home/michal/git/lidar_global_localisation/data/octomap_clouds/global/1719225215.109473530.pcd"
cloud_arr = loader.load_pcd_file_as_np(pcd_file, voxel_size = voxel_size)


pcd_file = "/home/michal/git/lidar_global_localisation/data/prior_clouds/5_2_2.pcd"
prior_cloud_arr = loader.load_pcd_file_as_np(pcd_file, voxel_size = voxel_size)

proc = CloudProcessing()

############################################
#heightmap
##############################################
heightmap = proc.create_height_map(cloud_arr, cell_size=pixel_size)
prior_heightmap = proc.create_height_map(prior_cloud_arr, cell_size=pixel_size)

#########################################
#density map
#########################################
#densitymap = proc.compute_2d_density_map_fixed(cloud_arr, cell_size=pixel_size)
#prior_densitymap = proc.compute_2d_density_map_fixed(prior_cloud_arr, cell_size=pixel_size)
#print(f"density map shape: {densitymap.shape}, max: {np.max(densitymap)}, min: {np.min(densitymap)}")


###############################################################
#### ORB
#############################################################
#keypoints, descriptors = proc.detect_orb_features(heightmap, nfeatures=500)
#proc.visualize_orb_features(heightmap, keypoints)


####################################
######## SIFT
#################################
keypoints, descriptors = proc.detect_sift_features(heightmap, nfeatures=500)
proc.visualize_sift_features(heightmap, keypoints)

######################################
#good features
########################################

#keypoints = proc.detect_good_features(heightmap, max_corners = 10000)
#proc.visualize_good_features(heightmap, keypoints)



#proc.visualize_multiple_2d_arrays([heightmap, prior_heightmap], titles=["heightmap", "prior_heightmap"], cmap='viridis')





##########################################
#grid search for harris corner detector
#########################################
"""
for k in [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.1, 0.2, 0.3]:
    for t in [0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]:
        for ks in [3, 5, 7]:
            keypoints = proc.detect_harris_corners(prior_heightmap,block_size=2, ksize=ks, k=k, threshold=t )
            print(f"{len(keypoints)} for {k=} {t=} {ks=}")

            if(len(keypoints)<10000 and len(keypoints)>1000):

                proc.visualize_harris_corners(prior_heightmap, keypoints)
"""

# Visualize Harris corners
#proc.visualize_harris_corners(heightmap, keypoints)

