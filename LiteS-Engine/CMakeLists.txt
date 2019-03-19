project (LiteS-Engine)



#
# Dependency Path
#
find_package(glfw3 CONFIG REQUIRED)
find_package(glew CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)
find_package(RapidJSON CONFIG REQUIRED)

include_directories (${GLFW3_INCLUDE_DIRS})
include_directories (${GLEW_INCLUDE_DIRS})
include_directories (${ASSIMP_INCLUDE_DIRS})
include_directories (${RAPIDJSON_INCLUDE_DIRS})

include_directories (resources/)

#
# Add source file
#
file(GLOB_RECURSE SOURCE_FILES src/*.cpp resources/*.cpp)
file(GLOB_RECURSE HEADER_FILES src/*.h resources/*.h)


add_library (LiteS ${SOURCE_FILES} ${HEADER_FILES})

#set_target_properties (LiteS PROPERTIES DEBUG_POSTFIX d)