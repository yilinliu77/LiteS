cmake_minimum_required (VERSION 3.1)
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "${PROJECT_BINARY_DIR}/scripts/buildsystems/vcpkg.cmake"
      CACHE STRING "")
endif()
project (LiteS)

# Compiler and Linker options
# Platform independent
if(${CMAKE_GENERATOR} MATCHES ".*Visual Studio.*|.*Ninja.*")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} /MD /Zi /Od /std:c++latest")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} /MD")

    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /Profile /LTCG")
elseif(${CMAKE_GENERATOR} MATCHES ".*Unix.*")
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -Wall -g -ggdb /std:c++latest")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -Wall")
else()
    message(FATAL_ERROR "Unsurpported generator")
endif(${CMAKE_GENERATOR} MATCHES ".*Visual Studio.*|.*Ninja.*")

find_package(glfw3 CONFIG REQUIRED)
find_package(glew CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)
find_package(RapidJSON CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)

include_directories (${GLFW3_INCLUDE_DIRS})
include_directories (${GLEW_INCLUDE_DIRS})
include_directories (${ASSIMP_INCLUDE_DIRS})
include_directories (${RAPIDJSON_INCLUDE_DIRS})
include_directories (${IMGUI_INCLUDE_DIRS})
include_directories (${GLM_INCLUDE_DIRS})

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/bin)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/bin)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/lib)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/lib)
SET(CMAKE_DEBUG_POSTFIX "_d")

ADD_SUBDIRECTORY(LiteS-Engine)
ADD_SUBDIRECTORY(LiteS-TestCase)