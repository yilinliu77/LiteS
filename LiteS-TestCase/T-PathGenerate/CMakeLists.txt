cmake_minimum_required(VERSION 3.8.0)
project(T-PathGenerate)

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_52,code=sm_52)

find_package(TBB CONFIG REQUIRED)

include_directories (${TBB_INCLUDE_DIRS})

file(GLOB SOURCE_FILES ./*.cpp)
file(GLOB HEADER_FILES ./*.h)
file(GLOB RESOURCES_FILES ./config.json ../../LiteS-Engine/resources/shaders/* ../../LiteS-Engine/resources/*)
include_directories (../../LiteS-Engine/resources/)
include_directories (../../LiteS-Engine/src/)

add_executable(T-PathGenerate ${SOURCE_FILES} ${HEADER_FILES} ${RESOURCES_FILES})
add_dependencies(T-PathGenerate LiteS)

target_link_libraries(T-PathGenerate PRIVATE TBB::tbb GLEW::GLEW glfw assimp::assimp imgui::imgui LiteS)
set_target_properties(T-PathGenerate PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

