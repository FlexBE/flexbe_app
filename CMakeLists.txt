cmake_minimum_required(VERSION 3.0)
project(flexbe_app)

find_package(catkin REQUIRED)

catkin_package()

#add_custom_target(electron_inst)
#add_custom_command(TARGET electron_inst POST_BUILD COMMAND bin/electron_install)

add_custom_command(OUTPUT electron
  COMMAND bin/electron_install
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  VERBATIM
)

add_custom_target(electron_install DEPENDS electron)

if(NOT ${CMAKE_CURRENT_SOURCE_DIR}/electron)
  safe_execute_process(COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/bin/electron_install)
endif()

#############
## Install ##
#############

install(PROGRAMS
    bin/electron_install
    bin/run_app
    bin/shortcut
    bin/test_report
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES
    package.json
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY
    src
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY
    electron
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
)
install(DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# run tests
if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	add_rostest(launch/test_report.test)
endif()
