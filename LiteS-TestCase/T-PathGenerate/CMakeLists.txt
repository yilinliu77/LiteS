cmake_minimum_required(VERSION 3.8.0)
project(T-PathGenerate)

#Path
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)


find_package(TBB CONFIG REQUIRED)

include_directories (${TBB_INCLUDE_DIRS})

link_directories(T-PathGenerate "d:/TOOL/vcpkg/packages/tbb_x64-windows/lib")
link_directories(T-PathGenerate "C:\\SDK\\vcpkg\\packages\\tbb_x64-windows\\lib")


# link_directories($ENV{OPENGL}/lib/x64)

file(GLOB SOURCE_FILES ./*.cpp)
file(GLOB HEADER_FILES ./*.h)
file(GLOB RESOURCES_FILES ./config.json ../../LiteS-Engine/resources/shaders/* ../../LiteS-Engine/resources/*)
include_directories (../../LiteS-Engine/resources/)
include_directories (../../LiteS-Engine/src/)

add_executable(T-PathGenerate ${SOURCE_FILES} ${HEADER_FILES} ${RESOURCES_FILES})
add_dependencies(T-PathGenerate LiteS)

target_link_libraries(T-PathGenerate PRIVATE TBB::tbb GLEW::GLEW glfw assimp::assimp imgui::imgui LiteS)

