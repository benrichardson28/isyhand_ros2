cmake_minimum_required(VERSION 3.8)
project(isyhand_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#find_package(isyhand_interface REQUIRED)

include_directories(include)

########## to create messages
set(msg_files
	"msg/JointState.msg"
	"msg/CommandJointPos.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

# after definitions before executables
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

############# One executable group ##############
add_executable(isyhand_driver_node 
	src/hardware_interface_node.cpp 
	src/hardware_interface.cpp
  include/isyhand_driver/hardware_interface.h
)
ament_target_dependencies(isyhand_driver_node
  rclcpp
  dynamixel_sdk
  std_msgs
)
#this is for messages defined in the same package
target_link_libraries(isyhand_driver_node "${cpp_typesupport_target}")
#################################################

############# One executable group ##############
add_executable(test_motion_node 
	src/test_motion.cpp
)
ament_target_dependencies(test_motion_node
 rclcpp
 std_msgs
)
target_link_libraries(test_motion_node "${cpp_typesupport_target}")
#################################################



# Install targets
install(TARGETS 
  isyhand_driver_node
  test_motion_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
# install(
#   DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )

# Install launch files if they exist
install(DIRECTORY
 launch/
 DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

