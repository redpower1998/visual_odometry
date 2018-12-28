SET(EXE_NAME visual_odometry_demo)

cmake_minimum_required(VERSION 2.4)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

find_package(PkgConfig)
pkg_check_modules(OpenCV opencv>=3.1.0)
set(OpenCV_LIBS ${OpenCV_LIBRARIES})
LINK_DIRECTORIES(${OpenCV_LIBDIR}) 

IF(WIN32) # Windows
    if (CMAKE_CL_64) 
        SET(OPENCV_DIR $ENV{OPENCV_DIR})
    else()
        message(FATAL_ERROR "You've selected the 32bit version of ${CMAKE_GENERATOR}. \n Please delete the cache (file->Delete Cache) and use the 64bit version. (${CMAKE_GENERATOR} Win64)")
    endif()
ELSE() # Linux
    add_definitions(-std=c++0x -O3 -pthread)
ENDIF(WIN32)

find_package(OpenCV ${VERSION_REQ_OCV} REQUIRED)
 
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${OpenCV_LIBRARY_DIRS})

SET(SRC_FOLDER src)
FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/*.cpp")

ADD_EXECUTABLE(${EXE_NAME} ${SRC_FILES})
target_link_libraries(${EXE_NAME}  ${OpenCV_LIBRARIES}  ${SPECIAL_OS_LIBS})
