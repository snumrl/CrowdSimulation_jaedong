cmake_minimum_required(VERSION 3.5)
project(csim)

add_compile_options(-std=c++11)

find_package(Boost 1.58.0 COMPONENTS filesystem regex python)
find_package(PythonInterp)
find_package(PythonLibs)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

file(GLOB srcs "srcs/src/simulator/*.cpp" "srcs/src/simulator/*.h" "srcs/src/simulator/scenario/*.cpp" "srcs/src/simulator/scenario/*.h")

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIR})

include(FindOpenMP)
if(OPENMP_FOUND)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/srcs/src)
add_library(csim SHARED ${srcs})
target_link_libraries(csim ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
set_target_properties(csim PROPERTIES PREFIX "" )

