cmake_minimum_required(VERSION 3.0)
project(feedback_bci_vr)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS 
             roscpp 
             std_msgs
             rosneuro_msgs
             )

find_library(NEUROCHRONO_LIBRARIES neurochrono REQUIRED)
find_path(NEUROCHRONO_INCLUDE_DIRS neurochrono/Timer.h REQUIRED)

find_path(SNDFILE_INCLUDE_DIR NAMES sndfile.h)
find_library(SNDFILE_LIBRARY NAMES sndfile)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    rosneuro_msgs
  DEPENDS
)


###########
## Build ##
###########

include_directories(include 
                    ${catkin_INCLUDE_DIRS}
                    ${NEUROCHRONO_INCLUDE_DIRS}
                    ${SNDFILE_INCLUDE_DIRS}
                    )

add_library(${PROJECT_NAME}    
            src/Training.cpp
            src/TrialSequence.cpp
            src/Autopilot.cpp
            )

target_link_libraries(${PROJECT_NAME} 
                      ${catkin_LIBRARIES}
                      ${NEUROCHRONO_LIBRARIES}
                      ${SNDFILE_LIBRARY}
                      )

add_dependencies(${PROJECT_NAME} 
                 ${catkin_EXPORTED_TARGETS}
                 # ${PROJECT_NAME}_gencfg
)

#################
## Executables ##
#################
add_executable(training_node src/training_node.cpp)
target_link_libraries(training_node  ${PROJECT_NAME}
                                    ${catkin_LIBRARIES}
                                    ${NEUROCHRONO_LIBRARIES}
                                    ${SNDFILE_LIBRARY}
                                    )