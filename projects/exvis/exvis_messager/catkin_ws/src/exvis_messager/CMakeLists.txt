cmake_minimum_required(VERSION 3.0.2)
project(exvis_messager)
set (CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)


set (GOROBOTS "../../../../../..")
set (UTILS "${GOROBOTS}/utils")
set (MAIN "${UTILS}/data_preprocessing/exvis_messager")

catkin_package(	LIBRARIES exvis_messager)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

include_directories(	include 
			${catkin_INCLUDE_DIRS}
			${MAIN})


add_executable(
	${PROJECT_NAME}_node 
	${MAIN}/exvis_messager
	)

add_dependencies(
	${PROJECT_NAME}_node 
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node  ${catkin_LIBRARIES})

