cmake_minimum_required(VERSION 3.1)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

project( DVL_Example )

find_package( OpenCV REQUIRED )
find_package( GTSAM REQUIRED )
find_package( Boost 1.50 REQUIRED )
#find_package( Eigen3 REQUIRED)

include_directories(${GTSAM_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})

set(GTSAM_LIBRARIES gtsam)

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

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


#add_executable( PressureFactor PressureFactor.cpp )
#gtsamAddExamplesGlob(  "*.cpp" "" "gtsam;${Boost_LIBS}" )
gtsamAddExamplesGlob(  "*.cpp" "" "gtsam;gtsam_unstable;${Boost_LIBS}" )
target_link_libraries( DVL_Example ${GTSAM_LIBS} ${Boost_LIBS} )
#{Eigen3_LIBS} )
