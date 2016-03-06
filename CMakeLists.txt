cmake_minimum_required (VERSION 2.6)
project (MoreMotionPlanning)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3")

add_executable(MoreMotionPlanning main.cpp)

#set_target_properties(MoreMotionPlanning PROPERTIES
#COMPILE_DEFINITIONS "STREAM_GRAPHICS")

set_target_properties(MoreMotionPlanning PROPERTIES
COMPILE_DEFINITIONS "IKFAST_NO_MAIN")

find_package(OMPL REQUIRED)
find_package(ASSIMP REQUIRED)
find_package(LAPACK REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(FCL REQUIRED fcl)

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.50 COMPONENTS system REQUIRED)

include_directories(
	${OMPL_INCLUDE_DIRS}
	${ASSIMP_LIBRARY_DIRS}
)

target_link_libraries(MoreMotionPlanning
	${OMPL_LIBRARIES}
	${OMPLAPP_LIBRARIES}
	${ASSIMP_LIBRARIES}
	${FCL_LIBRARIES}
	${Boost_LIBRARIES}
	${LAPACK_LIBRARIES}
)