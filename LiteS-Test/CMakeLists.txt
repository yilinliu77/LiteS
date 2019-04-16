cmake_minimum_required(VERSION 3.8.0)
project(LiteS-UnitTest)

find_package(glfw3 CONFIG REQUIRED)
find_package(glew CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(JPEG REQUIRED)
find_package(PNG REQUIRED)
find_package(TIFF REQUIRED)
find_package(GTest MODULE REQUIRED)
include_directories (${GLFW3_INCLUDE_DIRS})
include_directories (${GLEW_INCLUDE_DIRS})
include_directories (${ASSIMP_INCLUDE_DIRS})
include_directories (${GLM_INCLUDE_DIRS})
include_directories (${IMGUI_INCLUDE_DIRS})

# link_directories($ENV{OPENGL}/lib/x64)

file(GLOB SOURCE_FILES ./*.cpp)
file(GLOB HEADER_FILES ./*.h)
file(GLOB RESOURCES_FILES ./config.json ../LiteS-Engine/resources/shaders/* ../LiteS-Engine/resources/*)
include_directories (../LiteS-Engine/resources/)
include_directories (../LiteS-Engine/src/)

add_executable(LiteS-UnitTest ${SOURCE_FILES} ${HEADER_FILES} ${RESOURCES_FILES})
add_dependencies(LiteS-UnitTest LiteS)

target_link_libraries(LiteS-UnitTest GLEW::GLEW glfw assimp::assimp imgui::imgui LiteS PNG::PNG TIFF::TIFF ${JPEG_LIBRARIES} GTest::GTest GTest::Main)

