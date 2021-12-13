cmake_minimum_required(VERSION 3.0.2)
project(exvis_grab)
set (CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)


set (GOROBOTS "../../../../../..")
set (CONTROLLER "${GOROBOTS}/controllers/exvis")
set (UTILS "${GOROBOTS}/utils")
set (EZO_CONTROLLER "${CONTROLLER}/exvis_grab")
set (ANN_FRAMEWORK "${UTILS}/ann-framework")
set (ANN_LIB "${UTILS}/ann-library")

catkin_package(	LIBRARIES exvis_grab)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

include_directories(	include 
			${catkin_INCLUDE_DIRS}
			${ANN_FRAMEWORK}
			${ANN_LIB}
			${GOROBOTS}
			${EZO_CONTROLLER})


add_executable(
	${PROJECT_NAME}_node 
	${EZO_CONTROLLER}/exo_controller
	${EZO_CONTROLLER}/neural_controller
	${EZO_CONTROLLER}/exo_joint_module
	${EZO_CONTROLLER}/exo_joint_module_parameters
	${ANN_FRAMEWORK}/interpolator2d
	${ANN_FRAMEWORK}/ann
	${ANN_FRAMEWORK}/neuron
	${ANN_FRAMEWORK}/synapse
	${ANN_FRAMEWORK}/transferfunction
	${ANN_LIB}/pmn
	${ANN_LIB}/vrn
	${ANN_LIB}/so2cpg
	${ANN_LIB}/rbf
	${ANN_LIB}/dmp
	${ANN_LIB}/gradn
	)

add_dependencies(
	${PROJECT_NAME}_node 
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node  ${catkin_LIBRARIES})

