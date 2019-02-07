#
# Dependency Path
#
find_package(glfw3 CONFIG REQUIRED)
find_package(glew CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)

include_directories (${GLFW3_INCLUDE_DIRS})
include_directories (${GLEW_INCLUDE_DIRS})
include_directories (${ASSIMP_INCLUDE_DIRS})

include_directories (LiteS/resources/)


SET(CMAKE_INSTALL_PREFIX ${PROJECT_BINARY_DIR}/lib)
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release;MinSizeRel;RelWithDebInfo")

#
# Add source file
#
file(GLOB_RECURSE SOURCE_FILES src/*.cpp resources/*.cpp)
file(GLOB_RECURSE HEADER_FILES src/*.h resources/*.h)


add_library (LiteS ${SOURCE_FILES} ${HEADER_FILES})

#set_target_properties (LiteS PROPERTIES DEBUG_POSTFIX d)