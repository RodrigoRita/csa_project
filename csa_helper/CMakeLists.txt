cmake_minimum_required(VERSION 3.8)
project(csa_helper)

# Default to C++17 (required by Humble)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Agv.msg"
  "msg/Product.msg"
  "msg/Station.msg"
  "msg/Order.msg"
  "msg/ControllerResponse.msg"
  "srv/GetResources.srv"
  "srv/GetTransport.srv"
  "srv/StartSkills.srv"
  "srv/ProductResource.srv"
  "srv/RequestTransport.srv"
  "srv/SetProductColor.srv"
  "srv/ReserveStation.srv"
  "srv/GetProductTypes.srv"
  "action/ExecuteSkill.action"
  "action/SelectPair.action"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
