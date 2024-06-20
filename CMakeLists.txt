cmake_minimum_required(VERSION 3.22)
project(LidarLocalisation)

# Set the build type to Release
set(CMAKE_BUILD_TYPE Release)

# Find PCL package
find_package(PCL 1.8 REQUIRED)

# Include the external project
add_subdirectory(map_closures_cpp)

# Add your main project source files
add_executable(LidarLocalisation src/main.cpp src/DataLoader.cpp)

# Link the external project's library to your main executable
# Assuming the external project creates a library target named map_closures
target_link_libraries(LidarLocalisation PRIVATE map_closures ${PCL_LIBRARIES})

# Ensure include directories are correctly set
target_include_directories(LidarLocalisation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${PCL_INCLUDE_DIRS})
