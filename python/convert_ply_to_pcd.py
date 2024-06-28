import numpy as np
import matplotlib.pyplot as plt
from pypcd4 import PointCloud
import open3d as o3d


#import ply
source_path = "/media/michal/SSD_Michal/MRS/sprind/utm_clouds/5.ply"
#source_path = "/home/michal/git/LidarLocalisationProject/map_tiles_ply/5_2_3.ply"
#target_path = "/home/michal/git/LidarLocalisationProject/map_tiles_ply/5_2_3.pcd"
target_path = "/home/michal/git/sprind_data/clouds/prior_clouds/5.pcd"
pcd = o3d.io.read_point_cloud(source_path)
o3d.io.write_point_cloud(target_path, pcd)


#visualise
for x in [0, 1, 2, 3, 4]:
    for y in [0,1,2,3,4]:
        print("converting 5_"+str(x)+"_"+str(y)+".ply")
        source_path = "/home/michal/git/LidarLocalisationProject/map_tiles_ply/5_"+str(x)+"_"+str(y)+".ply"
        target_path = "/home/michal/git/sprind_data/clouds/5_"+str(x)+"_"+str(y)+".pcd"
        pcd = o3d.io.read_point_cloud(source_path)
        o3d.io.write_point_cloud(target_path, pcd)

#pc = PointCloud.from_path("/home/michal/git/LidarLocalisationProject/map_tiles_ply/5_2_3.pcd")
#pc_arr = pc.numpy(("x", "y", "z"))
#compute_2d_density_map_fixed(pc_arr, cell_size=1.0)
