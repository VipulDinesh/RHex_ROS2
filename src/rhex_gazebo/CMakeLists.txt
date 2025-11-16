cmake_minimum_required(VERSION 3.8)
project(rhex_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_gz REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(rhex_description REQUIRED)
find_package(xacro REQUIRED)

# Install directories (worlds, launch, models)
install(DIRECTORY
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()