cmake_minimum_required(VERSION 3.11.0)
project(T-VisualizeCameras)

#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_52,code=sm_52)

find_package(TBB CONFIG REQUIRED)

include_directories (${TBB_INCLUDE_DIRS})

file(GLOB SOURCE_FILES ./*.cpp)
file(GLOB HEADER_FILES ./*.h)
file(GLOB RESOURCES_FILES ./config.json ../../LiteS-Engine/resources/shaders/* ../../LiteS-Engine/resources/*)
include_directories (../../LiteS-Engine/resources/)
include_directories (../../LiteS-Engine/src/)
#include_directories (../../LiteS-CUDA/)

add_executable(T-VisualizeCameras ${SOURCE_FILES} ${HEADER_FILES} ${RESOURCES_FILES})
add_dependencies(T-VisualizeCameras LiteS)
#add_dependencies(T-VisualizeCameras LiteS-CUDA)

#target_link_libraries(T-VisualizeCameras PRIVATE TBB::tbb GLEW::GLEW glfw ${ASSIMP_LIBRARIES} imgui::imgui
#LiteS LiteS-CUDA)
target_link_libraries(T-VisualizeCameras PRIVATE TBB::tbb LiteS GLEW::GLEW  glfw imgui::imgui assimp::assimp)
#set_target_properties(T-VisualizeCameras PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

