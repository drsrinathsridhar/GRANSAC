CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(LineFittingSample)

IF(MSVC)
  ADD_DEFINITIONS(-DNOMINMAX)
  # Another MSVC quirk. http://stackoverflow.com/questions/6832666/lnk2019-when-including-asio-headers-solution-generated-with-cmake
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /MP") # For multi-threaded compilation on MSVC
ELSE()
  IF(APPLE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++") # C++11 support on Apple clang
  ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")# C++11 support on GCC
  ENDIF()
ENDIF()

FIND_PACKAGE(OpenCV 2.4 REQUIRED) # NEED OpenCV built with OpenGL support!
# OpenMP
FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  MESSAGE(STATUS "Found OpenMP")
ENDIF()

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/../include ${OpenCV_INCLUDE_DIRS})

SET(CPP_FILES LineFittingSample.cpp)
SET(VC_LIBS ${OpenCV_LIBS})
	
ADD_EXECUTABLE(${PROJECT_NAME} ${CPP_FILES} ${HPP_FILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${VC_LIBS})
