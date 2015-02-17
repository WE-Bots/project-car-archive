cmake_minimum_required(VERSION 2.8)
project( displayimage )
find_package( OpenCV REQUIRED )
add_executable( displayimage DisplayImage.cpp )
target_link_libraries( displayimage ${OpenCV_LIBS} )

