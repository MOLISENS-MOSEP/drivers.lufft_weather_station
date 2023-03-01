cmake_minimum_required(VERSION 3.8)
project(lufft_wsx_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LufftWSXXX.msg"
  "msg/Precipitation.msg"
  "msg/Temperature.msg"
  "msg/Humidity.msg"
  "msg/Enthalpy.msg"
  "msg/AirPressure.msg"
  "msg/AirDensity.msg"
  "msg/Wind.msg"
  "msg/Radiation.msg"

  DEPENDENCIES std_msgs
)

ament_package()
