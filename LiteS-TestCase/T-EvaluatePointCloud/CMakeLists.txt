cmake_minimum_required(VERSION 3.11.0)
project(T-EvaluatePointCloud)

find_package(TBB CONFIG REQUIRED)

include_directories (${TBB_INCLUDE_DIRS})

file(GLOB SOURCE_FILES ./*.cpp ./*.cu)
file(GLOB HEADER_FILES ./*.h)
file(GLOB RESOURCES_FILES ./config.json ../../LiteS-Engine/resources/shaders/* ../../LiteS-Engine/resources/*)
include_directories (../../LiteS-Engine/resources/)
include_directories (../../LiteS-Engine/src/)
include_directories (../../LiteS-CUDA/)

add_executable(T-EvaluatePointCloud ${SOURCE_FILES} ${HEADER_FILES} ${RESOURCES_FILES})

add_dependencies(T-EvaluatePointCloud LiteS)

target_link_libraries(T-EvaluatePointCloud LiteS 
TBB::tbb  GLEW::GLEW  glfw imgui::imgui assimp)
