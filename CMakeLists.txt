cmake_minimum_required (VERSION 3.13)
project (LiteS CXX)

set(CMAKE_TOOLCHAIN_FILE $ENV{vcpkg} CACHE STRING "" FORCE)
message(${CMAKE_TOOLCHAIN_FILE})

# Compiler and Linker options
# Platform independent
if(${CMAKE_GENERATOR} MATCHES ".*Visual Studio.*|.*Ninja.*")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} /MDd /Zi /Od")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} /MD")

    SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS} /Debug /Incremental")
    SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS} /LTCG")
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -G -Xcompiler \"/wd 4819\"")
elseif(${CMAKE_GENERATOR} MATCHES ".*Unix.*")
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -Wall -g -ggdb")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -Wall")
else()
    message(FATAL_ERROR "Unsurpported generator")
endif(${CMAKE_GENERATOR} MATCHES ".*Visual Studio.*|.*Ninja.*")

set (CMAKE_CXX_STANDARD 17)

find_package(glm CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glew CONFIG REQUIRED)
find_package(assimp REQUIRED)
find_package(RapidJSON CONFIG)
find_package(imgui CONFIG REQUIRED)
find_package(tinyply CONFIG REQUIRED)
include_directories (${GLFW3_INCLUDE_DIRS})
include_directories (${GLEW_INCLUDE_DIRS})
include_directories (${ASSIMP_INCLUDE_DIRS})
include_directories (${RAPIDJSON_INCLUDE_DIRS})
include_directories (${IMGUI_INCLUDE_DIRS})
include_directories (${GLM_INCLUDE_DIRS})
include_directories (${TINYPLY_INCLUDE_DIRS})

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/bin)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/bin)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/lib)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/lib)
SET(CMAKE_DEBUG_POSTFIX "_d")

ADD_SUBDIRECTORY(LiteS-Engine)
ADD_SUBDIRECTORY(LiteS-CUDA)
ADD_SUBDIRECTORY(LiteS-TestCase)
#ADD_SUBDIRECTORY(LiteS-Test)
