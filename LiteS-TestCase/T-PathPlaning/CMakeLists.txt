cmake_minimum_required(VERSION 3.8.0)
project(T-PathPlaning)

#Path
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

# Compiler and Linker options
# Platform independent
if(${CMAKE_GENERATOR} MATCHES ".*Visual Studio.*|.*Ninja.*")
	set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} /MDd /Zi /GL /Od")
	set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} /MD")

	SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /Profile /LTCG")
elseif(${CMAKE_GENERATOR} MATCHES ".*Unix.*")
	SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
	SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
else()
	message(FATAL_ERROR "Unsurpported generator")
endif(${CMAKE_GENERATOR} MATCHES ".*Visual Studio.*|.*Ninja.*")

find_package(glfw3 CONFIG REQUIRED)
find_package(glew CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
include_directories (${GLFW3_INCLUDE_DIRS})
include_directories (${GLEW_INCLUDE_DIRS})
include_directories (${ASSIMP_INCLUDE_DIRS})
include_directories (${GLM_INCLUDE_DIRS})
include_directories (${IMGUI_INCLUDE_DIRS})

# link_directories($ENV{OPENGL}/lib/x64)

file(GLOB SOURCE_FILES ./*.cpp ../../LiteS/src/*.cpp)
file(GLOB HEADER_FILES ./*.h ../../LiteS/src/*.h)
file(GLOB RESOURCES_FILES ./config.xml ../../LiteS/resources/shaders/* ../../LiteS/resources/*)
include_directories (../../LiteS/resources/)


#add_subdirectory(libs)

add_executable(T-PathPlaning ${SOURCE_FILES} ${HEADER_FILES} ${RESOURCES_FILES})

#add_dependencies(main tinyply)

target_link_libraries(T-PathPlaning GLEW::GLEW glfw assimp::assimp imgui::imgui)

