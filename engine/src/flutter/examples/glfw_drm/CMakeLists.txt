cmake_minimum_required(VERSION 3.15)
project(FlutterEmbedderGLFW)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )

add_executable(flutter_glfw FlutterEmbedderGLFW.cc)

############################################################
# GLFW
############################################################
option(GLFW_BUILD_EXAMPLES "" OFF)
option(GLFW_BUILD_TESTS "" OFF)
option(GLFW_BUILD_DOCS "" OFF)
option(GLFW_INSTALL "" OFF)
find_package(OpenGL REQUIRED COMPONENTS EGL)
include_directories(${OPENGL_INCLUDE_DIRS})
add_subdirectory(${CMAKE_SOURCE_DIR}/../../third_party/glfw glfw)
target_link_libraries(flutter_glfw glfw OpenGL::EGL)
include_directories(${CMAKE_SOURCE_DIR}/../../third_party/glfw/include)

############################################################
# Flutter Engine
############################################################
# This is assuming you've built a local version of the Flutter Engine.  If you
# downloaded yours is from the internet you'll have to change this.
include_directories(${CMAKE_SOURCE_DIR}/../../shell/platform/embedder)
find_library(FLUTTER_LIB flutter_engine PATHS ${CMAKE_SOURCE_DIR}/../../../out/host_debug_unopt)
target_link_libraries(flutter_glfw ${FLUTTER_LIB})

# Copy the flutter library here since the shared library
# name is `./libflutter_engine.dylib`.
add_custom_command(
        TARGET flutter_glfw POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
                ${FLUTTER_LIB}
                ${CMAKE_CURRENT_BINARY_DIR})
