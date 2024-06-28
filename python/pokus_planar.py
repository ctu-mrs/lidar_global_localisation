
import numpy as np
import matplotlib.pyplot as plt
from pypcd4 import PointCloud
import open3d as o3d
import cv2





pcd = o3d.io.read_point_cloud("/home/michal/git/sprind_data/clouds/local/1719225158.364459320.pcd")
pcd = pcd.voxel_down_sample(voxel_size=1.0)

pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=4.0, max_nn=20))

assert (pcd.has_normals())

'''
o3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024],
                                  point_show_normal=True)
'''
#o3d.visualization.draw_geometries([pcd])

# using all defaults
oboxes = pcd.detect_planar_patches(
    normal_variance_threshold_deg=50,
    coplanarity_deg=70,
    outlier_ratio=1000,
    min_plane_edge_length=0,
    min_num_points=0,
    search_param=o3d.geometry.KDTreeSearchParamKNN(knn=20))

print("Detected {} patches".format(len(oboxes)))

geometries = []
for obox in oboxes:
    mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox, scale=[1, 1, 0.0001])
    mesh.paint_uniform_color(obox.color)
    geometries.append(mesh)
    geometries.append(obox)
geometries.append(pcd)

o3d.visualization.draw_geometries(geometries,
                                  zoom=0.62,
                                  front=[0.4361, -0.2632, -0.8605],
                                  lookat=[2.4947, 1.7728, 1.5541],
                                  up=[-0.1726, -0.9630, 0.2071])
