cmake_minimum_required(VERSION 3.11.0)
project(T-PathGenerate LANGUAGES CUDA CXX)

enable_language(CUDA)

include(CheckLanguage)
check_language(CUDA)


set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler -std=cxx11")
set(CMAKE_CUDA_STANDARD 11)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_CUDA_RESOLVE_DEVICE_SYMBOLS ON)
set(CUDA_SEPARABLE_COMPILATION ON)

find_package(TBB CONFIG REQUIRED)

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
