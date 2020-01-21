cmake_minimum_required(VERSION 2.8)
project( FeatureDetection )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

file( GLOB MAIN_SOURCES *.cpp )
file( GLOB MAIN_HEADERS *.h )

add_executable( FeatureDetect ${MAIN_SOURCES} ${MAIN_HEADERS} )
target_link_libraries( FeatureDetect ${OpenCV_LIBS} )
