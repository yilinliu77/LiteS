cmake_minimum_required(VERSION 3.11.0)
project(T-PathGenerate LANGUAGES CUDA CXX)

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_52,code=sm_52;-std=c++11)
LIST(APPEND CUDA_NVCC_FLAGS "-Xcompiler \"/wd 4819\"")



find_package(TBB CONFIG REQUIRED)
find_package(CUDA REQUIRED)
enable_language(CUDA)
set(CUDA_LIBRARIES PRIVATE ${CUDA_LIBRARIES})

include_directories (${TBB_INCLUDE_DIRS})

file(GLOB SOURCE_FILES ./*.cpp ./*.cu)
file(GLOB HEADER_FILES ./*.h)
file(GLOB RESOURCES_FILES ./config.json ../../LiteS-Engine/resources/shaders/* ../../LiteS-Engine/resources/*)
include_directories (../../LiteS-Engine/resources/)
include_directories (../../LiteS-Engine/src/)
include_directories (../../LiteS-CUDA/)

add_executable(T-PathGenerate ${SOURCE_FILES} ${HEADER_FILES} ${RESOURCES_FILES})

add_dependencies(T-PathGenerate LiteS)
add_dependencies(T-PathGenerate LiteS-CUDA)

target_link_libraries(T-PathGenerate PRIVATE TBB::tbb GLEW::GLEW glfw assimp::assimp imgui::imgui LiteS LiteS-CUDA)
set_target_properties(T-PathGenerate PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_property(TARGET T-PathGenerate PROPERTY CUDA_STANDARD 11)
