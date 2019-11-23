cmake_minimum_required(VERSION 3.11.0)
project(T-PathGenerate LANGUAGES CUDA CXX)

enable_language(CUDA)

include(CheckLanguage)
check_language(CUDA)

find_package(OPENMP REQUIRED)

#include_directories (${TBB_INCLUDE_DIRS})

file(GLOB SOURCE_FILES ./*.cpp ./*.cu)
file(GLOB HEADER_FILES ./*.h)
file(GLOB RESOURCES_FILES ./config.json ../../LiteS-Engine/resources/shaders/* ../../LiteS-Engine/resources/*)
include_directories (../../LiteS-Engine/resources/)
include_directories (../../LiteS-Engine/src/)
include_directories (../../LiteS-CUDA/)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")

add_executable(T-PathGenerate ${SOURCE_FILES} ${HEADER_FILES} ${RESOURCES_FILES})

add_dependencies(T-PathGenerate LiteS)
add_dependencies(T-PathGenerate LiteS-CUDA)

target_link_libraries(T-PathGenerate PRIVATE LiteS LiteS-CUDA GLEW::GLEW glfw 
assimp::assimp imgui::imgui tinyply)

set_target_properties(T-PathGenerate PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(T-PathGenerate PROPERTIES RESOLVE_DEVICE_SYMBOLS ON)
set_target_properties(T-PathGenerate PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties(T-PathGenerate PROPERTIES CUDA_STANDARD 11)
