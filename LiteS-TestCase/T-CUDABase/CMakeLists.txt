cmake_minimum_required(VERSION 3.14.0)
project(T-CUDABase LANGUAGES CUDA CXX)

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_52,code=sm_52)
LIST(APPEND CUDA_NVCC_FLAGS "-Xcompiler \"/wd 4819\"")
add_definitions(/wd"4819" /wd"4996" /wd"4800")

find_package(glfw3 CONFIG REQUIRED)
find_package(glew CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
include_directories (${GLFW3_INCLUDE_DIRS})
include_directories (${GLEW_INCLUDE_DIRS})
include_directories (${ASSIMP_INCLUDE_DIRS})
include_directories (${GLM_INCLUDE_DIRS})
include_directories (${IMGUI_INCLUDE_DIRS})

file(GLOB SOURCE_FILES ./*.cpp ./*.cu)
file(GLOB HEADER_FILES ./*.h)
file(GLOB RESOURCES_FILES ./config.json ../../LiteS-Engine/resources/shaders/* ../../LiteS-Engine/resources/*)
include_directories (../../LiteS-Engine/resources/)
include_directories (../../LiteS-Engine/src/)

add_executable(T-CUDABase ${SOURCE_FILES} ${HEADER_FILES} ${RESOURCES_FILES})
add_dependencies(T-CUDABase LiteS)

target_link_libraries(T-CUDABase GLEW::GLEW glfw assimp::assimp imgui::imgui LiteS)
set_target_properties(T-CUDABase PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
