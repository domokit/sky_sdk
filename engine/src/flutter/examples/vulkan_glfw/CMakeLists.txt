cmake_minimum_required(VERSION 3.15)
project(FlutterEmbedderVulkanGLFW)

set(CMAKE_CXX_STANDARD 17)

# Primary target

set(EXE_NAME embedder_example_vulkan)

file(GLOB_RECURSE SOURCE_FILES "src/*.cc" "src/*.h")
add_executable(${EXE_NAME} ${SOURCE_FILES})


# Dependency: Vulkan SDK

# Override the SDK location by using the VULKAN_SDK environment variable.
# The VULKAN_SDK path should contain `include/vulkan/vulkan.hpp`.
#
# MacOS MoltenVK Vulkan SDK environment example:
#  export VULKAN_SDK=~/VulkanSDK/1.2.198.1/macOS
#  export VK_LOADER_DEBUG=all
#  export VK_ICD_FILENAMES=$VULKAN_SDK/share/vulkan/icd.d/MoltenVK_icd.json
#  export VK_LAYER_PATH=$VULKAN_SDK/share/vulkan/explicit_layer.d
#  export DYLD_LIBRARY_PATH=$VULKAN_SDK/lib:$DYLD_LIBRARY_PATH

find_package(Vulkan REQUIRED)
target_include_directories(${EXE_NAME} PRIVATE ${Vulkan_INCLUDE_DIRS})
target_link_libraries(${EXE_NAME} PRIVATE Vulkan::Vulkan)


# Dependency: GLFW

set(GLFW_REPOSITORY ${CMAKE_SOURCE_DIR}/../../third_party/glfw)

set(BUILD_SHARED_LIBS OFF)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_VULKAN_STATIC OFF CACHE BOOL "" FORCE)
add_subdirectory(${GLFW_REPOSITORY} build_glfw)

target_link_libraries(${EXE_NAME} PRIVATE glfw)
target_include_directories(${EXE_NAME} PRIVATE ${GLFW_REPOSITORY}/include)


# Dependency: Flutter Engine

# This is assuming you've built a local version of the Flutter Engine. If you
# downloaded yours from the internet you'll have to change this.
set(EMBEDDER_H_DIR ${CMAKE_SOURCE_DIR}/../../shell/platform/embedder)
set(FLUTTER_OUT_DIR ${CMAKE_SOURCE_DIR}/../../../out/host_debug_unopt)

find_library(FLUTTER_LIB flutter_engine PATHS ${FLUTTER_OUT_DIR})

target_link_libraries(${EXE_NAME} PRIVATE ${FLUTTER_LIB})
target_include_directories(${EXE_NAME} PRIVATE ${EMBEDDER_H_DIR})

# Copy the flutter library here since the shared library
# name is `./libflutter_engine.dylib`.
add_custom_command(
        TARGET ${EXE_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
                ${FLUTTER_LIB}
                ${CMAKE_CURRENT_BINARY_DIR})
