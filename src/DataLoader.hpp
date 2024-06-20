// include/DataLoader.hpp
#ifndef DATALOADER_HPP
#define DATALOADER_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

class DataLoader {
public:
    
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> parsePLYFile(const std::string &filename);
    void convertPointCloudToEigenVector(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                        std::vector<Eigen::Vector3d> &eigen_vector);
};

#endif // DATALOADER_HPP
