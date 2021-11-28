cmake_minimum_required(VERSION 2.8)

project(Controller)

find_package(Boost COMPONENTS system thread filesystem)

add_definitions(-std=c++14)

include(FindPkgConfig)
pkg_check_modules(cnoid_plugin REQUIRED choreonoid-body-plugin)

include_directories(
	${cnoid_plugin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

link_directories(
	${cnoid_plugin_LIBRARY_DIRS}
	${Boost_LIBRARY_DIRS}
)

set(MOTION_ENGINE_SRC
	MotionEngine.hpp MotionEngine.cpp
	Kinematics.hpp Kinematics.cpp
	utils.hpp
)

add_library(Controller SHARED Controller.cpp ${MOTION_ENGINE_SRC})
set_target_properties(Controller PROPERTIES PREFIX "")

set(target Controller)
target_link_libraries(Controller
	${cnoid_plugin_LIBRARIES}
	${Boost_LIBRARIES}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

install(TARGETS Controller LIBRARY DESTINATION /usr/local/lib/choreonoid-1.8/simplecontroller)
