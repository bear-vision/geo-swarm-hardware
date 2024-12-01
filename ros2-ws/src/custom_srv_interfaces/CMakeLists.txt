cmake_minimum_required(VERSION 3.8)
project(custom_srv_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclpy REQUIRED)  # For Python clients/servers

find_package(rosidl_default_generators REQUIRED)
#add dependencies that are used in rosidl_generate_interfaces below
find_package(geometry_msgs REQUIRED) 

# get all srv files
set(SRVS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/srv")
file(GLOB CUSTOM_SRVS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${SRVS_DIR}/*.srv")

# get all msg files
set(MSGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB CUSTOM_MSGS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${MSGS_DIR}/*.msg")

# get all action files
set(ACTIONS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/action")
file(GLOB CUSTOM_ACTIONS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${ACTIONS_DIR}/*.action")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${CUSTOM_SRVS}
  ${CUSTOM_MSGS}
  ${CUSTOM_ACTIONS}
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for path planner msgs
)

# Install Python modules and generated interface files
install(PROGRAMS
  # Python executables can be added here, e.g., `scripts/my_python_node.py`
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
