cmake_minimum_required(VERSION 3.8)
project(walker_s2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  meshes
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  ../../doc/LICENSES/Apache-2.0.txt
  ../../doc/LICENSES/README.md
  ../../doc/LICENSES/UBTECH-PROPRIETARY.txt
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
