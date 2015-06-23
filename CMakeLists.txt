cmake_minimum_required(VERSION 2.6)
project(CRFnew)
find_package(PkgConfig REQUIRED)
# configure opencv
pkg_check_modules(OpenCV opencv)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


include_directories(include/Eigen)
include_directories( ../external/liblbfgs/include ../external/ )

add_library( lbfgs external/liblbfgs/lib/lbfgs.c )
add_library( densecrf src/util.cpp src/permutohedral.cpp src/unary.cpp src/pairwise.cpp src/objective.cpp src/densecrf.cpp src/labelcompatibility.cpp )

add_library( optimization src/optimization.cpp )
target_link_libraries( optimization lbfgs )

include_directories(include)




#TEST
set(SOURCE_FILES_TEST testCRF.cpp ppm.cpp src/densecrf.cpp )#include/common.h include/densecrf.h)
add_executable(CRFTest ${SOURCE_FILES_TEST} settings.h ppm.h CRFTest.h)
target_link_libraries(CRFTest densecrf optimization ${OpenCV_LDFLAGS})

set(SOURCE_FILES_TRAIN testTrainCRF.cpp ppm.cpp src/densecrf.cpp )#include/common.h include/densecrf.h)
add_executable(CRFTrain ${SOURCE_FILES_TRAIN} settings.h ppm.h CRFTrain.h)
target_link_libraries(CRFTrain  densecrf optimization ${OpenCV_LDFLAGS})


#BOOST
FIND_PACKAGE(Boost 1.54.0)
IF(Boost_FOUND)
  INCLUDE_DIRECTORIES("${Boost_INCLUDE_DIRS}" "/usr/include/python2.7")
  SET(Boost_USE_STATIC_LIBS OFF)
  SET(Boost_USE_MULTITHREADED ON)
  SET(Boost_USE_STATIC_RUNTIME OFF)
  FIND_PACKAGE(Boost 1.54.0 COMPONENTS python)
  ADD_LIBRARY(yay SHARED dummy.cpp)
  TARGET_LINK_LIBRARIES(yay ${Boost_LIBRARIES})
ELSEIF(NOT Boost_FOUND)
  MESSAGE(FATAL_ERROR "Unable to find correct Boost version. Did you set BOOST_ROOT?")
ENDIF()