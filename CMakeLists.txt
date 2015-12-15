cmake_minimum_required (VERSION 2.6)
project (MoreMotionPlanning)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3")
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -g")

add_executable(MoreMotionPlanning main.cpp)

find_package(OMPL REQUIRED)
find_package(ASSIMP REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(FCL REQUIRED fcl)

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.46.0 COMPONENTS thread system REQUIRED)

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
)