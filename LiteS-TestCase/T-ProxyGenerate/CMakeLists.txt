cmake_minimum_required(VERSION 3.8.0)
project(T-ProxyGenerate)

find_package(glfw3 CONFIG REQUIRED)
find_package(glew CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(JPEG REQUIRED)
find_package(PNG REQUIRED)
find_package(TIFF REQUIRED)
include_directories (${GLFW3_INCLUDE_DIRS})
include_directories (${GLEW_INCLUDE_DIRS})
include_directories (${ASSIMP_INCLUDE_DIRS})
include_directories (${GLM_INCLUDE_DIRS})
include_directories (${IMGUI_INCLUDE_DIRS})

# link_directories($ENV{OPENGL}/lib/x64)

file(GLOB SOURCE_FILES ./*.cpp)
file(GLOB HEADER_FILES ./*.h)
file(GLOB RESOURCES_FILES ./config.xml ../../LiteS-Engine/resources/shaders/* ../../LiteS-Engine/resources/*)
include_directories (../../LiteS-Engine/resources/)
include_directories (../../LiteS-Engine/src/)


#add_subdirectory(libs)
include_directories (C:/Users/vcc/Documents/repo/SFM/REFERENCE/mve/libs)
link_directories (C:/Users/vcc/Documents/repo/SFM/REFERENCE/mve/libs/fssr/Release/)
link_directories (C:/Users/vcc/Documents/repo/SFM/REFERENCE/mve/libs/mve/Release/)
link_directories (C:/Users/vcc/Documents/repo/SFM/REFERENCE/mve/libs/util/Release/)


add_executable(T-ProxyGenerate ${SOURCE_FILES} ${HEADER_FILES} ${RESOURCES_FILES})
add_dependencies(T-ProxyGenerate LiteS)

target_link_libraries(T-ProxyGenerate GLEW::GLEW glfw assimp::assimp imgui::imgui LiteS mve_fssr.lib mve.lib mve_util.lib PNG::PNG TIFF::TIFF ${JPEG_LIBRARIES})

