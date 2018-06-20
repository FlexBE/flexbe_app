cmake_minimum_required(VERSION 2.8.3)
project(flexbe_app)

find_package(catkin REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

# specify catkin-specific information
# INCLUDE_DIRS - The exported include paths (i.e. cflags) for the package
# LIBRARIES - The exported libraries from the project
# CATKIN_DEPENDS - Other catkin projects that this project depends on
# DEPENDS - Non-catkin CMake projects that this project depends on
# CFG_EXTRAS - Additional configuration options 
catkin_package(
    INCLUDE_DIRS src
    LIBRARIES ${PROJECT_NAME})

# use add_library() or add_executable() as required
#add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})

#FILE(GLOB BIN_FILES "bin/*")

# install executables
#install(PROGRAMS ${BIN_FILES} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#add_custom_target(nwjs_inst)
#add_custom_command(TARGET nwjs_inst POST_BUILD COMMAND bin/nwjs_install)

add_custom_command(OUTPUT nw
  COMMAND bin/nwjs_install
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  VERBATIM
)

add_custom_target(nw_install DEPENDS nw)

if(NOT ${CMAKE_CURRENT_SOURCE_DIR}/nw)
  safe_execute_process(COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/bin/nwjs_install)
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# call catkin_destinations() to populate CATKIN_*_DESTINATION cmake variables
catkin_destinations()

install(DIRECTORY
  launch
  src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  bin/run_app
  bin/shortcut
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  lib
  locales
  swiftshader
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES
  credits.html
  flexbe.desktop
  icudtl.dat
  natives_blob.bin
  nw_100_percent.pak
  nw_200_percent.pak
  package.json
  resources.pak
  snapshot_blob.bin
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  nw
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
