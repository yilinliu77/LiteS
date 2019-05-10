cmake_minimum_required(VERSION 3.11.0)
project(T-RemovePointInside)

find_package(TBB CONFIG REQUIRED)

include_directories (${TBB_INCLUDE_DIRS})

file(GLOB SOURCE_FILES ./*.cpp ./*.cu)
file(GLOB HEADER_FILES ./*.h)
file(GLOB RESOURCES_FILES ./config.json ../../LiteS-Engine/resources/shaders/* ../../LiteS-Engine/resources/*)
include_directories (../../LiteS-Engine/resources/)
include_directories (../../LiteS-Engine/src/)
include_directories (../../LiteS-CUDA/)

add_executable(T-RemovePointInside ${SOURCE_FILES} ${HEADER_FILES} ${RESOURCES_FILES})

add_dependencies(T-RemovePointInside LiteS)

target_link_libraries(T-RemovePointInside assimp::assimp 
TBB::tbb LiteS GLEW::GLEW  glfw imgui::imgui)