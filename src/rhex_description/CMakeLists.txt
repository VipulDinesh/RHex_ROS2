cmake_minimum_required(VERSION 3.8)
project(rhex_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)

# Install model resources
install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.urdf"
  PATTERN "*.xacro"
  PATTERN "*.stl"
  PATTERN "*.gazebo"
  PATTERN "*.trans"
  PATTERN "*.launch.py"
  PATTERN "*.rviz"
)

ament_package()

