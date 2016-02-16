cmake_minimum_required (VERSION 2.6)
project (MoreMotionPlanning)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3")

add_executable(Visualizer main.cpp)

find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
INCLUDE(FindOpenGL)
INCLUDE(FindGLEW)

find_package(ASSIMP REQUIRED)

include_directories(
	${GLFW_INCLUDE_DIRS}
	${GLEW_DIRS}
	${OPENGL_INCLUDE_DIRS}
	${ASSIMP_LIBRARY_DIRS}
)

target_link_libraries(Visualizer
	"/usr/local/lib/libassimp.dylib"
)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

	FIND_LIBRARY(COCOA_LIBRARY Cocoa)
	FIND_LIBRARY(COREVIDEO_LIBRARY CoreVideo)
	FIND_LIBRARY(IOKIT_LIBRARY IOKit)

	target_link_libraries(Visualizer
		"/usr/local/lib/libglfw3.a"
		${GLEW_LIBRARIES}
		${OPENGL_LIBRARIES}
		${COCOA_LIBRARY}
		${COREVIDEO_LIBRARY}
		${IOKIT_LIBRARY}
	)
else(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	target_link_libraries(Visualizer
		${GLFW_STATIC_LIBRARIES}
		${GLEW_LIBRARIES}
		${OPENGL_LIBRARIES}
	)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")