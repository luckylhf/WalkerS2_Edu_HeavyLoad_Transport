cmake_minimum_required(VERSION 3.8)
project(box_grasp_demo)

if(CMAKE_CROSSCOMPILING)
  message(FATAL_ERROR
    "Bundled ubt_robot wheels support native x86_64/aarch64 builds only; "
    "cross-compilation is not supported")
endif()

# The navigation workflow uses UBT's binary CPython 3.10 SDK.  Keep it inside
# this workspace instead of mutating the system Python installation.
find_program(UBT_ROBOT_PYTHON_EXECUTABLE
  NAMES python3.10 python3
  PATHS /usr/bin /usr/local/bin
  NO_DEFAULT_PATH
  REQUIRED
)
execute_process(
  COMMAND "${UBT_ROBOT_PYTHON_EXECUTABLE}" -c
          "import sys; raise SystemExit(0 if sys.version_info[:2] == (3, 10) else 1)"
  RESULT_VARIABLE UBT_ROBOT_PYTHON_VERSION_OK
)
if(NOT UBT_ROBOT_PYTHON_VERSION_OK EQUAL 0)
  message(FATAL_ERROR
    "ubt_robot requires CPython 3.10, but ${UBT_ROBOT_PYTHON_EXECUTABLE} is incompatible")
endif()

# Keep all ROS Python packages in this workspace ABI-compatible with the
# bundled SDK.  This also overrides an activated conda interpreter.
set(PYTHON_EXECUTABLE "${UBT_ROBOT_PYTHON_EXECUTABLE}" CACHE FILEPATH
  "Python interpreter used by ROS interface generators" FORCE)
set(Python3_EXECUTABLE "${UBT_ROBOT_PYTHON_EXECUTABLE}" CACHE FILEPATH
  "Python3 interpreter used by CMake packages" FORCE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(action_msgs REQUIRED)
find_package(box_grasp_demo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

find_file(UBT_ROBOT_MOSQUITTO_LIBRARY
  NAMES libmosquitto.so.1
  PATHS
    "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
    "/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
    /usr/lib /lib
  NO_DEFAULT_PATH
)
if(NOT UBT_ROBOT_MOSQUITTO_LIBRARY)
  message(WARNING
    "libmosquitto.so.1 was not found. The navigation workflow will not run until "
    "'sudo apt install libmosquitto1' is executed on the target computer.")
endif()

ament_python_install_package(
  box_grasp_demo PACKAGE_DIR src/box_grasp_demo SKIP_COMPILE)

install(PROGRAMS
  scripts/box_grasp_node_ros2.py
  scripts/fake_box_scene_ros2.py
  scripts/recorded_frame_publisher.py
  scripts/grasp_validation_ros2.py
  scripts/arm_ik_pinocchio.py
  scripts/head_tf_bridge.py
  scripts/joint_state_bridge.py
  scripts/arm_pose_gui.py
  scripts/trigger_detect_box.py
  scripts/trigger_detect_box_sim.py
  scripts/navigate_box_workflow.py
  scripts/calibrate_table_height.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(DIRECTORY vendor/ubt_robot/
  DESTINATION share/${PROJECT_NAME}/vendor/ubt_robot
)

install(FILES
  ../../doc/LICENSES/Apache-2.0.txt
  ../../doc/LICENSES/CC0-1.0.txt
  ../../doc/LICENSES/EDL-1.0-mosquitto.txt
  ../../doc/LICENSES/MIT-cpp-tbox.txt
  ../../doc/LICENSES/MIT-nlohmann-json.txt
  ../../doc/LICENSES/README.md
  ../../doc/LICENSES/SDK-THIRD-PARTY-NOTICES.md
  ../../doc/LICENSES/UBTECH-PROPRIETARY.txt
  DESTINATION share/${PROJECT_NAME}
)

# Run during CMake's install phase (part of `colcon build`).  The helper picks
# the wheel matching the host architecture and installs it below this package's
# isolated/merged prefix without invoking pip or accessing the network.
install(CODE "
  execute_process(
    COMMAND \"${UBT_ROBOT_PYTHON_EXECUTABLE}\"
            \"${CMAKE_CURRENT_SOURCE_DIR}/cmake/install_ubt_robot_wheel.py\"
            --wheel-dir \"${CMAKE_CURRENT_SOURCE_DIR}/vendor/ubt_robot\"
            --prefix \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}\"
    RESULT_VARIABLE UBT_ROBOT_WHEEL_INSTALL_RESULT
  )
  if(NOT UBT_ROBOT_WHEEL_INSTALL_RESULT EQUAL 0)
    message(FATAL_ERROR \"Failed to install the bundled ubt_robot wheel\")
  endif()
")

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/ubt_robot_pythonpath.dsv.in"
)

ament_package()
