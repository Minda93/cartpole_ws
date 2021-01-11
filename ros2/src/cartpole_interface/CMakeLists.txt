cmake_minimum_required(VERSION 3.5)
project(cartpole_interface)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cartpole_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

include_directories(include)

# =====
# create components
# ======

add_library(cartpole_interface SHARED
  src/cartpole_interface.cpp)
target_compile_definitions(cartpole_interface
  PRIVATE "CARTPOLE_INTERFACE_BUILDING_DLL")
ament_target_dependencies(cartpole_interface
  "rclcpp"
  "rclcpp_components"
  "std_srvs"
  "std_msgs"
  "cartpole_msgs"
  "gazebo_msgs")
rclcpp_components_register_nodes(cartpole_interface "interface::CartpoleInterface")

#=============
# Install
#=============

# add the install(TARGETS…) section so ros2 can find your component
install(TARGETS
  cartpole_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
