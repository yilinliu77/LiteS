cmake_minimum_required(VERSION 3.14.0)
project(T-CUDABase LANGUAGES CUDA CXX)

#find_package(CUDA REQUIRED)
enable_language(CUDA)

include(CheckLanguage)
check_language(CUDA)

#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_52,code=sm_52)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler \"/wd 4819\"" )
set(CMAKE_CUDA_STANDARD 11)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_EXTENSIONS ON)

set(CUDA_SEPARABLE_COMPILATION ON)

#find_package(glfw3 CONFIG REQUIRED)
#find_package(glew CONFIG REQUIRED)
#find_package(assimp CONFIG REQUIRED)
#find_package(imgui CONFIG REQUIRED)
#find_package(glm CONFIG REQUIRED)
#include_directories (${GLFW3_INCLUDE_DIRS})
#include_directories (${GLEW_INCLUDE_DIRS})
#include_directories (${ASSIMP_INCLUDE_DIRS})
#include_directories (${GLM_INCLUDE_DIRS})
#include_directories (${IMGUI_INCLUDE_DIRS})

file(GLOB SOURCE_FILES ./*.cpp ./*.cu)
file(GLOB HEADER_FILES ./*.h)
#file(GLOB RESOURCES_FILES ./config.json ../../LiteS-Engine/resources/shaders/* ../../LiteS-Engine/resources/*)
#include_directories (../../LiteS-Engine/resources/)
#include_directories (../../LiteS-Engine/src/)

add_executable(T-CUDABase ${SOURCE_FILES} ${HEADER_FILES} ${RESOURCES_FILES})

#target_link_libraries(T-CUDABase GLEW::GLEW glfw assimp::assimp imgui::imgui)
#set_target_properties(T-CUDABase PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
