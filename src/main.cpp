#include "map_closures/DensityMap.hpp" 
#include "map_closures/MapClosures.hpp"
#include "DataLoader.hpp"

#include <iostream>

int main() {
    // Your main project code
    std::cout<<"Hello C++"<<std::endl;

    // define map closures object
    map_closures::Config config;
    config.density_map_resolution = 1.0;
    config.density_threshold = 0.2;
    config.hamming_distance_threshold = 50;
    map_closures::MapClosures map_closures(config);
    int id = 0;

    //load data
    DataLoader dataLoader;
    std::string folderPath = "/home/michal/git/PointCloudLoader/ply_files/tiles_ply/";
    std::vector<std::string> files = {"5_0_0.ply", "5_0_1.ply", "5_0_2.ply", "5_0_3.ply", "5_0_4.ply","5_1_0.ply", "5_1_1.ply", "5_1_2.ply", "5_1_3.ply", "5_1_4.ply"};

    for (const std::string& file : files) {
      std::string filePath = folderPath + file;
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud = dataLoader.parsePLYFile(filePath);     
      
      if (cloud) {
          std::cout << "Point cloud "<< file << " loaded successfully!" << std::endl;
      } else {
          std::cout << "Failed to load point cloud." << std::endl;
      }
      
      //downsample the pointcloud
      float filter_voxel_size = 0.25;
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> down_cloud = dataLoader.downsamplePointCloud(cloud, filter_voxel_size);

      std::vector<Eigen::Vector3d> pointVector;
      dataLoader.convertPointCloudToEigenVector(down_cloud, pointVector);
      std::cout<<pointVector.size()<<std::endl;
      
      //for (const auto &point : pointVector) {
      //      std::cout << "Point: [" << point.x() << ", " << point.y() << ", " << point.z() << "]" << std::endl;
      //}
      
      //add submap to database
      map_closures.AddNewSubmap(id, pointVector);
      std::cout<< "Added submap with id "<<std::to_string(id)<<std::endl; 
      
      //visualise the submap (save it to file)
      //std::string outputFilePath = "/home/michal/git/LidarLocalisationProject/debug/grid_" + std::to_string(id) + ".png";
      //map_closures.saveSubmapToFile(id, outputFilePath);

      //query match of new
      //pointcloud
      id++;
    }

    return 0;
}

