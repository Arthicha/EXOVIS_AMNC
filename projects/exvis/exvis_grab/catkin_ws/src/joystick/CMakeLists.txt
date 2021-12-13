
cmake_minimum_required(VERSION 3.0.2)
project(exvis_joystick)
set (CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)


set (GOROBOTS "../../../../../..")
set (CONTROLLER "${GOROBOTS}/controllers/exvis")
set (UTILS "${GOROBOTS}/utils")
set (EZO_CONTROLLER "${CONTROLLER}/exvis_joystick")

catkin_package(	LIBRARIES exvis_joystick)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

include_directories(	include 
			${catkin_INCLUDE_DIRS}
			${GOROBOTS}
			${EZO_CONTROLLER})


add_executable(
	${PROJECT_NAME}_node 
	${EZO_CONTROLLER}/joystick_teleop
	${EZO_CONTROLLER}/joystick
	)

add_dependencies(
	${PROJECT_NAME}_node 
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node  ${catkin_LIBRARIES})

