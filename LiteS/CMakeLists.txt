#
# Compiler and Linker options
# Platform independent
#
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

#
# Dependency Path
#
find_package(glfw3 CONFIG REQUIRED)
find_package(glew CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)

include_directories (${GLFW3_INCLUDE_DIRS})
include_directories (${GLEW_INCLUDE_DIRS})
include_directories (${ASSIMP_INCLUDE_DIRS})

SET(CMAKE_INSTALL_PREFIX ${PROJECT_BINARY_DIR}/lib)
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release;MinSizeRel;RelWithDebInfo")

#
# Add source file
#
file(GLOB_RECURSE SOURCE_FILES src/*.cpp)
file(GLOB_RECURSE HEADER_FILES src/*.h)

add_library (LiteS ${SOURCE_FILES} ${HEADER_FILES})

#set_target_properties (LiteS PROPERTIES DEBUG_POSTFIX d)