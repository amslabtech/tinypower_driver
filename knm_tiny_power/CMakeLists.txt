cmake_minimum_required(VERSION 2.8.3)
project(knm_tiny_power)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -O2")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cereal_catkin
  knm_tiny_msgs
  nav_msgs
  geometry_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  #CATKIN_DEPENDS
  #DEPENDS
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)


add_library(knm_tiny_lib src/knm_tiny_lib.cpp)
add_dependencies(knm_tiny_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(knm_tiny_lib
  ${catkin_LIBRARIES}
)

add_executable(knm_tiny_joy src/knm_tiny_joy.cpp)
add_dependencies(knm_tiny_joy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(knm_tiny_joy
  ${catkin_LIBRARIES}
)

add_executable(mcnk_tiny_joy src/mcnk_tiny_joy.cpp)
add_dependencies(mcnk_tiny_joy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mcnk_tiny_joy
  ${catkin_LIBRARIES}
)

add_executable(mcnk_cheat_joy src/mcnk_cheat_joy.cpp)
add_dependencies(mcnk_cheat_joy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mcnk_cheat_joy
  ${catkin_LIBRARIES}
)

add_executable(knm_tiny_power src/knm_tiny_power.cpp)
add_dependencies(knm_tiny_power ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(knm_tiny_power knm_tiny_lib)

add_executable(tf_for_bag src/tf_for_bag.cpp)
add_dependencies(tf_for_bag ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_for_bag
  ${catkin_LIBRARIES}
)

add_executable(knm_tiny_kari src/knm_tiny_kari.cpp)
add_dependencies(knm_tiny_kari ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(knm_tiny_kari
  ${catkin_LIBRARIES}
)


#add_executable(knm_tiny_power_calibration src/knm_tiny_power_calibration.cpp)
#target_link_libraries(knm_tiny_power_calibration cereal_catkin knm_tiny_lib)

