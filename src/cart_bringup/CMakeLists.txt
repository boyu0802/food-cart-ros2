cmake_minimum_required(VERSION 3.8)
project(cart_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

# Python node(s) shipped with this ament_cmake package. Installed into
# lib/<pkg> (dropping the .py) so `ros2 run cart_bringup scan_sector_filter`
# and launch_ros `executable='scan_sector_filter'` both resolve it.
install(PROGRAMS
  scripts/scan_sector_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME scan_sector_filter
)

ament_package()
