project (LiteS-Engine)



#
# Dependency Path
#

include_directories (resources/)

#
# Add source file
#
file(GLOB_RECURSE SOURCE_FILES src/*.cpp resources/*.cpp)
file(GLOB_RECURSE HEADER_FILES src/*.h resources/*.h)


add_library (LiteS ${SOURCE_FILES} ${HEADER_FILES})

#set_target_properties (LiteS PROPERTIES DEBUG_POSTFIX d)