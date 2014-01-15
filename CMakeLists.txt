cmake_minimum_required(VERSION 2.8.3)
project(survey_uwsim)
find_package(catkin REQUIRED COMPONENTS std_msgs rospy roscpp tf geometry_msgs)

find_package( OpenSceneGraph REQUIRED COMPONENTS osgSim osgUtil osgDB osgFX osgGA osgTerrain osgViewer osgText osgWidget osg)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(survey_trajectory src/makeTrajectory.cpp)
TARGET_LINK_LIBRARIES(survey_trajectory ${OPENSCENEGRAPH_LIBRARIES} ${catkin_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS std_msgs rospy roscpp tf geometry_msgs
)
