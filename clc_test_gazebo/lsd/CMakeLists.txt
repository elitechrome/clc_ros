cmake_minimum_required(VERSION 2.8)
project( DisplayImage )
find_package( OpenCV REQUIRED )
link_directories(/opt/intel/composerxe/ipp/lib/intel64
/opt/intel/composerxe/lib/intel64

)
add_library(lsd lsd.c)
add_executable( lsd_opencv lsd_opencv.cpp )
target_link_libraries( lsd_opencv lsd ${OpenCV_LIBS} )
