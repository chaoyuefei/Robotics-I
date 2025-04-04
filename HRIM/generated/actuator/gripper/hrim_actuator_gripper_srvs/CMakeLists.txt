cmake_minimum_required(VERSION 3.5)

project(hrim_actuator_gripper_srvs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
	"srv/ControlFinger.srv"
	"srv/ControlVacuum.srv"
	"srv/GoalGripper.srv"
	"srv/SpecsFingerGripper.srv"
	"srv/SpecsMagneticGripper.srv"
	"srv/SpecsVacuumGripper.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
