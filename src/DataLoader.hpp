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
    
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> parsePLYFile(const std::string &filename);
    void convertPointCloudToEigenVector(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                        std::vector<Eigen::Vector3d> &vec);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> downsamplePointCloud(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud, float leaf_size);
};

#endif // DATALOADER_HPP
