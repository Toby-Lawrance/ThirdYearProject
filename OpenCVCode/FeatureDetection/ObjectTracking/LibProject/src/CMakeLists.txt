find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${ObjectTracking_SOURCE_DIR}/LibProject/inc)
add_library(LibProject main.cpp)