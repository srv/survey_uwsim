cmake_minimum_required(VERSION 2.8.3)
project(survey_uwsim)
find_package(catkin REQUIRED COMPONENTS std_msgs rospy roscpp tf geometry_msgs)

find_package( OpenSceneGraph REQUIRED COMPONENTS osgSim osgUtil osgDB osgFX osgGA osgTerrain osgViewer osgText osgWidget osg)

catkin_package(
  CATKIN_DEPENDS std_msgs rospy roscpp tf geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(survey_trajectory src/makeTrajectory.cpp)
target_link_libraries(survey_trajectory ${OPENSCENEGRAPH_LIBRARIES} ${catkin_LIBRARIES})
add_executable(make_survey src/make_survey.cpp)
target_link_libraries(make_survey ${OPENSCENEGRAPH_LIBRARIES} ${catkin_LIBRARIES})
