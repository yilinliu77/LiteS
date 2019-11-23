set(CMAKE_TOOLCHAIN_FILE $ENV{vcpkg} CACHE STRING "" FORCE)
message(STATUS ${CMAKE_TOOLCHAIN_FILE})
cmake_minimum_required (VERSION 3.13)
project (LiteS CXX)

if(NOT DEFINED ${CMAKE_BUILD_TYPE})
    set(CMAKE_BUILD_TYPE Debug)
endif(NOT DEFINED ${CMAKE_BUILD_TYPE})

set (CMAKE_CXX_STANDARD 17)

# Compiler and Linker options
# Platform independent
if(${CMAKE_GENERATOR} MATCHES ".*Visual Studio.*|.*Ninja.*")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} /MDd /Zi /Od")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} /MD")

    SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS} /Debug /Incremental")
    SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS} /LTCG")
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -G -Xcompiler \"/wd 4819\"")
    set(CMAKE_CUDA_FLAGS "-Xcudafe --diag_suppress=esa_on_defaulted_function_ignored ${CMAKE_CUDA_FLAGS} ")
elseif(${CMAKE_GENERATOR} MATCHES ".*Unix.*")
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -pthread -O0 -Wall -g -ggdb -std=c++1z")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -Wall -pthread -std=c++1z")

    SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS}")
    SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS}")
else()
    message(FATAL_ERROR "Unsurpported generator")
endif(${CMAKE_GENERATOR} MATCHES ".*Visual Studio.*|.*Ninja.*")



find_package(glm CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glew CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)

find_package(RapidJSON CONFIG)
find_package(imgui CONFIG REQUIRED)
# find_package(tinyply CONFIG REQUIRED)
include_directories (${GLFW3_INCLUDE_DIRS})
include_directories (${GLEW_INCLUDE_DIRS})
include_directories (${ASSIMP_INCLUDE_DIRS})
include_directories (${RAPIDJSON_INCLUDE_DIRS})
include_directories (${IMGUI_INCLUDE_DIRS})
include_directories (${GLM_INCLUDE_DIRS})
# include_directories (${TINYPLY_INCLUDE_DIRS})

link_directories(${ASSIMP_ROOT_DIR}/x86_64-linux-gnu)
#message(FATAL_ERROR ${ASSIMP_LIBRARIES_DIRS})

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/bin)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/bin)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/lib)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/lib)
SET(CMAKE_DEBUG_POSTFIX "_d")

SET(LITES_CUDA_ON 0)

ADD_SUBDIRECTORY(LiteS-Engine)
if (LITES_CUDA_ON)
    ADD_SUBDIRECTORY(LiteS-CUDA)
endif(LITES_CUDA_ON)
ADD_SUBDIRECTORY(LiteS-TestCase)
#ADD_SUBDIRECTORY(LiteS-Test)
