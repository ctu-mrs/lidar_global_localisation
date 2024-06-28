import numpy as np
import matplotlib.pyplot as plt
from pypcd4 import PointCloud
import open3d as o3d
import cv2
import ground_removal_ext

def remove_ground_plane(pcd, distance_treshold = 1.0, num_iter = 300):
    pc = np.asarray(pcd.points)
    print(f"[ground removal] input cloud shape {pc.shape}")
    # ground removal
    segmentation = ground_removal_ext.ground_removal_kernel(pc, distance_treshold, num_iter) # distance_th=0.2, iter=200
    
    # (25000, 5), the last channel represents if this point is ground,
    # 0: ground, 255: Non-ground
    print(f"[ground removal] removed {len(segmentation[:, 4]>254)}")
    pc = segmentation[segmentation[:, 4]<254] 
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc[:, :3])
    return pcd










pcd = o3d.io.read_point_cloud("/home/michal/git/sprind_data/clouds/prior_clouds/5_3_1.pcd")





pcd = pcd.voxel_down_sample(voxel_size=0.1)
"""
pc = np.asarray(pcd.points)
print(pc.shape)
# ground removal
segmentation = ground_removal_ext.ground_removal_kernel(pc, 2.0, 500) # distance_th=0.2, iter=200

print(segmentation.shape)
print(segmentation[:, 4])
pc = segmentation[segmentation[:, 4]<254] 

print(pc.shape)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pc[:, :3])
# (25000, 5), the last channel represents if this point is ground,
# 0: ground, 255: Non-ground

"""
pcd = remove_ground_plane(
o3d.visualization.draw_geometries([pcd])
