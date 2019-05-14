cmake_minimum_required(VERSION 3.13.0)
project(GenerateNadir)

file(GLOB SOURCE_FILES ./*.cpp)
file(GLOB HEADER_FILES ./*.h)
file(GLOB RESOURCES_FILES ./config.json ../../../LiteS-Engine/resources/shaders/* ../../../LiteS-Engine/resources/*)
include_directories (../../../LiteS-Engine/resources/)
include_directories (../../../LiteS-Engine/src/)

add_executable(GenerateNadir ${SOURCE_FILES} ${HEADER_FILES} ${RESOURCES_FILES})
add_dependencies(GenerateNadir LiteS)

target_link_libraries(GenerateNadir PRIVATE GLEW::GLEW glfw ${ASSIMP_LIBRARIES} imgui::imgui LiteS)

