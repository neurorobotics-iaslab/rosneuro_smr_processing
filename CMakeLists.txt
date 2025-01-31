cmake_minimum_required(VERSION 3.0.2)
project(rosneuro_smr_processing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  Eigen3
  rosneuro_buffers
  rosneuro_buffers_ringbuffer
  rosneuro_decoder
  rosneuro_filters
  rosneuro_filters_laplacian
  rosneuro_msgs
  rosneuro_pwelch
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  ${PROJECT_NAME}
	
  CATKIN_DEPENDS
  	roscpp
	  roslib
	  std_msgs
	  rosneuro_filters
    rosneuro_pwelch
    rosneuro_filters_laplacian
    rosneuro_buffers_ringbuffer
    rosneuro_decoder
    rosneuro_msgs
  DEPENDS
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(smr src/smr.cpp)
target_link_libraries(smr ${catkin_LIBRARIES} fftw3)


