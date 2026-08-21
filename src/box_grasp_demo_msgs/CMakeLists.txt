cmake_minimum_required(VERSION 3.8)
project(box_grasp_demo_msgs)

# DetectBox is imported by the navigation process together with UBT's cp310
# binary SDK, so its generated Python extension must use the same ABI even if
# colcon is launched from an activated conda environment.
find_program(BOX_GRASP_PYTHON_EXECUTABLE
  NAMES python3.10 python3
  PATHS /usr/bin /usr/local/bin
  NO_DEFAULT_PATH
  REQUIRED
)
execute_process(
  COMMAND "${BOX_GRASP_PYTHON_EXECUTABLE}" -c
          "import sys; raise SystemExit(0 if sys.version_info[:2] == (3, 10) else 1)"
  RESULT_VARIABLE BOX_GRASP_PYTHON_VERSION_OK
)
if(NOT BOX_GRASP_PYTHON_VERSION_OK EQUAL 0)
  message(FATAL_ERROR
    "box_grasp_demo navigation requires CPython 3.10, but "
    "${BOX_GRASP_PYTHON_EXECUTABLE} is incompatible")
endif()
set(PYTHON_EXECUTABLE "${BOX_GRASP_PYTHON_EXECUTABLE}" CACHE FILEPATH
  "Python interpreter used by ROS interface generators" FORCE)
set(Python3_EXECUTABLE "${BOX_GRASP_PYTHON_EXECUTABLE}" CACHE FILEPATH
  "Python3 interpreter used by CMake packages" FORCE)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DetectBox.action"
  "srv/ComputePlaceIK.srv"
  "srv/SuggestCrouch.srv"
  DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
