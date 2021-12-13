cmake_minimum_required(VERSION 3.0.2)
project(exvis_interface)
set (CMAKE_CXX_STANDARD 11)


set (GOROBOTS "../../../../../..")
set (ROOT "${GOROBOTS}/../../../..")
set (UTILS "${GOROBOTS}/utils")
set (LIBS "${GOROBOTS}/../../../../usr/lib")
set (REALROBOT "${UTILS}/real_robots/exvis/exvis_real")




find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

catkin_package(	LIBRARIES exvis_interface)

include_directories(include ${catkin_INCLUDE_DIRS} ${GOROBOTS} ${REALROBOT} )

add_executable(
	${PROJECT_NAME}_node 
	${REALROBOT}/ExvisCommuCANMain
	${REALROBOT}/ExvisCommuCANAssem
	${REALROBOT}/ExvisCommuCANROS
	${REALROBOT}/CommuCAN
	)



add_dependencies(
	${PROJECT_NAME}_node 
	${catkin_EXPORTED_TARGETS}
)


link_directories(${LIBS})

target_link_libraries(${PROJECT_NAME}_node  ${catkin_LIBRARIES} pcanbasic)


