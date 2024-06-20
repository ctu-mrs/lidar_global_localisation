#include "map_closures/DensityMap.hpp" // Adjust the path as necessary
#include "map_closures/MapClosures.hpp" // Adjust the path as necessary
#include <iostream>

int main() {
    // Your main project code
    std::cout<<"Hello C++"<<std::endl;


    map_closures::MapClosures map_closures;
    int id = 0;
    const std::vector<Eigen::Vector3d> vec;
    map_closures.JustAdd(id, vec);
    return 0;
}

