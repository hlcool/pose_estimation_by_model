cmake_minimum_required(VERSION 2.8)

project(walker2_pose)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-pthread ${CMAKE_CXX_FLAGS}")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/libs)

# 设置ID解码宏定义开关
option(USE_CIRCLE "Set to switch to build use CIRCLE" OFF) 
option(USE_RECT "Set to switch to build use RECT" ON) 
if (USE_CIRCLE) 
	add_definitions(-DUSE_CIRCLE) # 宏定义 USE_CIRCLE 
	message(STATUS "Use CIRCLE") 
elseif(USE_RECT) 
	add_definitions(-DUSE_RECT) # 宏定义 USE_RECT 
	message(STATUS "Use RECT") 
else() 
	message(STATUS "Use Nothing") 
endif()


find_package(OpenCV 3.4 REQUIRED)
link_directories(${OpenCV_LIB_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include
					${PROJECT_SOURCE_DIR}/include/astra
					${PROJECT_SOURCE_DIR}/include/astra/astra_include)

link_directories(${PROJECT_SOURCE_DIR}/libs
                 ${PROJECT_SOURCE_DIR}/libs/astra
				 ${PROJECT_SOURCE_DIR}/libs/astra/OpenNI2/Drivers)

add_subdirectory(${PROJECT_SOURCE_DIR}/libmoped)
add_subdirectory(${PROJECT_SOURCE_DIR}/src)
