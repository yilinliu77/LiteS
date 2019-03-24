cmake_minimum_required(VERSION 3.8.0)
project(T-FeatureDetectSIFT)


file(GLOB SOURCE_FILES ./*.cpp)
file(GLOB HEADER_FILES ./*.h)
file(GLOB RESOURCES_FILES ./config.json)
include_directories (../../LiteS-Engine/resources/)
include_directories (../../LiteS-Engine/src/)

add_executable(T-FeatureDetectSIFT ${SOURCE_FILES} ${HEADER_FILES})
add_dependencies(T-FeatureDetectSIFT LiteS)

target_link_libraries(T-FeatureDetectSIFT LiteS)

