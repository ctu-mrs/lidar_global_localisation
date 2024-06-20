cmake_minimum_required(VERSION 3.22)
project(LidarLocalisation)

# Set the build type to Release
set(CMAKE_BUILD_TYPE Release)

# Include the external project
add_subdirectory(map_closures_cpp)

# Add your main project source files
add_executable(LidarLocalisation src/main.cpp)

# Link the external project's library to your main executable
# Assuming the external project creates a library target named map_closures
target_link_libraries(LidarLocalisation PRIVATE map_closures)
