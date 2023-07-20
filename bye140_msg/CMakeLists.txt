cmake_minimum_required(VERSION 3.5)
project(bye140_msg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ByStatus.msg"
  "srv/CalibrateGripper.srv"
  "srv/GetCalibrated.srv"
  "srv/MoveTo.srv"
  "srv/RestartGripper.srv"
  "srv/ShutdownGripper.srv"
)

ament_package()
