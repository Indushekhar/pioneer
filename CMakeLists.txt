cmake_minimum_required(VERSION 2.8.3)
project(pioneer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	sensor_msgs
	cv_bridge
	roscpp
        rospy
	geometry_msgs
	message_generation
	image_transport
        std_msgs
        message_runtime
)

## Generate added messages and services
add_service_files(
	FILES 
	velocityChangeService.srv 
	motionService.srv
	captureImageService.srv
)


generate_messages(DEPENDENCIES std_msgs )


catkin_package()



find_package( OpenCV REQUIRED COMPONENTS core imgproc highgui )
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(pioneer src/main.cpp src/Robot.cpp  src/Exploration.cpp src/RobotCamera.cpp )
target_link_libraries(pioneer ${catkin_LIBRARIES} ${OpenCV_LIBS})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0") # debug, no optimisation
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage") # enabling coverage

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(pioneerTest test/pioneer_test.launch test/main.cpp test/RobotTest.cpp test/ExplorationTest.cpp test/RobotCameraTest.cpp src/Robot.cpp  src/Exploration.cpp src/RobotCamera.cpp )
  target_link_libraries(pioneerTest ${catkin_LIBRARIES} ${OpenCV_LIBS})
  add_dependencies(
	  pioneerTest
	  pioneer
	  ${catkin_EXPORTED_TARGETS}
)
endif()

