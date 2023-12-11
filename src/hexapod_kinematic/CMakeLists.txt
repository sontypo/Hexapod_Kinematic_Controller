cmake_minimum_required(VERSION 3.0.2)
project(hexapod_kinematic)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  geometry_msgs
  hexapod_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hexapod_kinematic
  CATKIN_DEPENDS nodelet geometry_msgs hexapod_msgs roscpp sensor_msgs std_msgs tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library( control src/control.cpp )
add_dependencies( control hexapod_msgs_generate_messages_cpp )

add_library( gait src/gait.cpp )
add_dependencies( gait hexapod_msgs_generate_messages_cpp )

add_library( ik src/ik.cpp )
add_dependencies( ik hexapod_msgs_generate_messages_cpp )

add_executable( hexapod_kinematic src/hexapod_kinematic.cpp )
target_link_libraries( hexapod_kinematic ${catkin_LIBRARIES} control gait ik )
add_dependencies( hexapod_kinematic hexapod_msgs_generate_messages_cpp )

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES hexapod_kinematic.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/get_link_state.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)