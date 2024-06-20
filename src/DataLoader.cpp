// src/DataLoader.cpp
#include "DataLoader.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

void DataLoader::convertPointCloudToEigenVector(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud, std::vector<Eigen::Vector3d> &pointVector) {
    pointVector.clear();
    pointVector.reserve(cloud->points.size()); // Reserve space for efficiency

    for (const auto& point : cloud->points) {
        Eigen::Vector3d vec(point.x, point.y, point.z);
        pointVector.push_back(vec);
    }
}


std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> DataLoader::parsePLYFile(const std::string &filename) {

    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Couldn't open file " << filename << std::endl;
        return nullptr;
    }

    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string line;
    bool headerEnded = false;
    int vertexCount = 0;
    int vertexIndex = 0;

    // Read header
    while (std::getline(infile, line) && !headerEnded) {
        std::istringstream iss(line);
        std::string word;
        iss >> word;
        if (word == "end_header") {
            headerEnded = true;
        } else if (word == "element" && iss >> word && word == "vertex") {
            iss >> vertexCount;
        }
    }

    // Read points
    while (std::getline(infile, line) && vertexIndex < vertexCount) {
        std::istringstream iss(line);
        pcl::PointXYZ point;
        double x, y, z;
        float scalarIntensity, scalarReturnNumber, scalarNumberOfReturns, scalarClassification;
        float scalarScanAngleRank, scalarUserData, scalarPointSourceID, scalarGpsTime;

        // Parse the line
        if (iss >> x >> y >> z >> scalarIntensity >> scalarReturnNumber >> scalarNumberOfReturns 
                >> scalarClassification >> scalarScanAngleRank >> scalarUserData >> scalarPointSourceID >> scalarGpsTime) {
            point.x = x;
            point.y = y;
            point.z = z;
            cloud->points.push_back(point);
            vertexIndex++;
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;
}

