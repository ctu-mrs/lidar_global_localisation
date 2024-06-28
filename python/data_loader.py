import numpy as np
import matplotlib.pyplot as plt
from pypcd4 import PointCloud
import open3d as o3d
import cv2

class DataLoader:
    def __init__(self):
        pass
    
    def load_pcd_file_as_np(self, file_path, voxel_size = 0.0):
        print(f"loading np pointcloud file from {file_path}")
        pcd = o3d.io.read_point_cloud(file_path)
        if(voxel_size>0):
            print(f"downsampling with voxel size {voxel_size}")
            pcd = self._downsample_cloud(pcd, voxel_size) 
        arr = np.asarray(pcd.points)
        print(f"shape: {arr.shape}\n")
        return arr

    def _downsample_cloud(self, pcd, voxel_size):
        downpcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        return downpcd
    
if __name__ == "__main__":
    loader = DataLoader()
    pcd_file = "/home/michal/git/sprind_data/clouds/global/1719225305.811924468.pcd"
    cloud_arr = loader.load_pcd_file_as_np(pcd_file, voxel_size = 0.5)










