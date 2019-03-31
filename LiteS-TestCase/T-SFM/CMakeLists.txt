cmake_minimum_required(VERSION 3.8.0)
project(T-SFM)


file(GLOB SOURCE_FILES ./*.cpp)
file(GLOB HEADER_FILES ./*.h)
file(GLOB RESOURCES_FILES ./config.json)
include_directories (../../LiteS-Engine/resources/)
include_directories (../../LiteS-Engine/src/)

find_path(FREEIMAGE_INCLUDE_DIR NAMES FreeImage.h)
include_directories(${FREEIMAGE_INCLUDE_DIR})
find_library(FREEIMAGE_LIBRARY FreeImaged)
find_package(OpenCV REQUIRED)

include_directories (${OpenCV_INCLUDE_DIRS})
include_directories (${TBB_INCLUDE_DIRS})


add_executable(T-SFM ${SOURCE_FILES} ${HEADER_FILES})
add_dependencies(T-SFM LiteS)

target_link_libraries(T-SFM LiteS ${OpenCV_LIBS} ${FREEIMAGE_LIBRARY} )
 
