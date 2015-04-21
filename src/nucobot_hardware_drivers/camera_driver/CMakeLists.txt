cmake_minimum_required(VERSION 2.8.3)
project(camera_driver)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport geometry_msgs
                                        pcl_conversions pcl_ros
                                        sensor_msgs std_msgs roscpp tf)
find_package(OpenCV REQUIRED)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS} 
                    ${OpenCV_INCLUDE_DIRS}
                    aruco)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


FILE(GLOB hdrs "./aruco/*.h*")
FILE(GLOB srcs "./aruco/*.c*")
SET(hdrs ${hdrs})
SET(srcs ${srcs})


#ADD_LIBRARY(${PROJECT_NAME} ${srcs} ${hdrs})


#add_library(... $<TARGET_OBJECTS:objlib> ...)

set(SRC_FILES "camera_driver.cpp aruco/ar_omp.cpp
                                 aruco/arucofidmarkers.cpp
                                 aruco/board.cpp
                                 aruco/boarddetector.cpp
                                 aruco/cameraparameters.cpp
                                 aruco/chromaticmask.cpp
                                 aruco/cvdrawingutils.cpp
                                 aruco/highlyreliablemarkers.cpp
                                 aruco/marker.cpp
                                 aruco/markerdetector.cpp
                                 aruco/subpixelcorner.cpp")

add_executable(camera_driver camera_driver.cpp ${srcs} ${hdrs})
target_link_libraries(camera_driver ${catkin_LIBRARIES} 
                                    ${OpenCV_LIBS}
                                    ${PCL_LIBRARIES})


