cmake_minimum_required(VERSION 3.16)
project(xela_ah_r2c_bringup)

find_package(ament_cmake REQUIRED)

# Install directories for launch files, configs, URDF/xacro, and scripts.
install(DIRECTORY
  launch
  config
  urdf
  mesh
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
)

# Register the package resource for ros2 launch/ament index queries.
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
