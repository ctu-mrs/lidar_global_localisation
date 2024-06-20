#include "map_closures/DensityMap.hpp" 
#include "map_closures/MapClosures.hpp"
#include "DataLoader.hpp"

#include <iostream>

int main() {
    // Your main project code
    std::cout<<"Hello C++"<<std::endl;

    DataLoader dataLoader;
    std::string folderPath = "/home/michal/git/PointCloudLoader/ply_files/tiles_ply/";
    std::vector<std::string> files = {"5_0_0.ply", "5_0_1.ply", "5_0_2.ply", "5_0_3.ply", "5_0_4.ply"};

    for (const std::string& file : files) {
      std::string filePath = folderPath + file;
      //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = dataLoader.parsePLYFile(filePath);
      std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud = dataLoader.parsePLYFile(filePath);     
      
      if (cloud) {
          std::cout << "Point cloud "<< file << " loaded successfully!" << std::endl;
      } else {
          std::cout << "Failed to load point cloud." << std::endl;
      }
      
      //std::vector<Eigen::Vector3d> pointVector;
      //dataLoader.convertPointCloudToEigenVector(cloud, pointVector);
      //std::cout<<pointVector.size()<<std::endl;
    }


    map_closures::MapClosures map_closures;
    int id = 0;
    const std::vector<Eigen::Vector3d> vec;
    return 0;
}

