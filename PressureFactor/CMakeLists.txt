cmake_minimum_required(VERSION 3.1)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

project( PressureFactor )

find_package( OpenCV REQUIRED )
find_package( GTSAM REQUIRED )
# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")


message(STATUS "GTSAM library status:")
message(STATUS "    config: ${GTSAM_DIR}")
message(STATUS "    version: ${GTSAM_VERSION}")
message(STATUS "    libraries: ${GTSAM_LIBS}")
message(STATUS "    include path: ${GTSAM_INCLUDE_DIRS}")


add_executable( PressureFactor PressureFactor.cpp )
target_link_libraries( PressureFactor ${GTSAM_LIBS} )
