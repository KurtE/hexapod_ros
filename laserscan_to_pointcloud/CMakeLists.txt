cmake_minimum_required(VERSION 2.8.3)
project(laserscan_to_pointcloud)

find_package( catkin REQUIRED COMPONENTS nodelet roscpp geometry_msgs laser_geometry tf ) 
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laserscan_to_pointcloud 
  CATKIN_DEPENDS nodelet roscpp geometry_msgs 
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable( laserscan_to_pointcloud src/laserscan_to_pointcloud.cpp ) 
target_link_libraries( laserscan_to_pointcloud ${catkin_LIBRARIES} )

install(TARGETS laserscan_to_pointcloud
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY sounds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/sounds
)
install(FILES laserscan_to_pointcloud.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
 