cmake_minimum_required(VERSION 3.8.0)
project(T-BVHDisplay)

find_package(TBB CONFIG REQUIRED)

include_directories (${TBB_INCLUDE_DIRS})

#Path
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

# link_directories($ENV{OPENGL}/lib/x64)

file(GLOB SOURCE_FILES ./*.cpp)
file(GLOB HEADER_FILES ./*.h)
file(GLOB RESOURCES_FILES ./config.json ../../LiteS-Engine/resources/shaders/* ../../LiteS-Engine/resources/*)
include_directories (../../LiteS-Engine/resources/)
include_directories (../../LiteS-Engine/src/)

add_executable(T-BVHDisplay ${SOURCE_FILES} ${HEADER_FILES} ${RESOURCES_FILES})
add_dependencies(T-BVHDisplay LiteS)

message(STATUS  ${ASSIMP_INCLUDE_DIRS})

target_link_libraries(T-BVHDisplay LiteS GLEW::GLEW glfw imgui::imgui assimp)

