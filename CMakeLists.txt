cmake_minimum_required(VERSION 3.5)
project(flexbe_app)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# use add_library() or add_executable() as required
#add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})

#add_custom_target(nwjs_inst)
#add_custom_command(TARGET nwjs_inst POST_BUILD COMMAND bin/nwjs_install)

add_custom_command(OUTPUT nwjs
  COMMAND bin/nwjs_install
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  VERBATIM
)

add_custom_target(nw_install DEPENDS nwjs)

#if(NOT ${CMAKE_CURRENT_SOURCE_DIR}/nwjs)
#  safe_execute_process(COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/bin/nwjs_install)
#endif()

#############
## Install ##
#############

INSTALL(PROGRAMS
    bin/run_app
    bin/shortcut
    bin/nwjs_install
    bin/test_report
    bin/find_flexbe_packages
    DESTINATION lib/${PROJECT_NAME}
)
INSTALL(FILES
    package.json
    DESTINATION lib/${PROJECT_NAME}
)
INSTALL(FILES
    flexbe.desktop
    DESTINATION lib/${PROJECT_NAME}
)
INSTALL(DIRECTORY
    src
    DESTINATION lib/${PROJECT_NAME}
)
INSTALL(DIRECTORY
    launch
    DESTINATION lib/${PROJECT_NAME}
)
INSTALL(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)
#INSTALL(DIRECTORY
#    nwjs
#    DESTINATION lib/${PROJECT_NAME}
#    USE_SOURCE_PERMISSIONS
#)

ament_package()
