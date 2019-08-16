project (LiteS-Engine)

if(${LITES_CUDA_ON})
	enable_language(CUDA)
	include(CheckLanguage)
	check_language(CUDA)
endif(${LITES_CUDA_ON})

include_directories (resources/)

configure_file(
		"${PROJECT_SOURCE_DIR}/src/config.h.in"
		"${PROJECT_BINARY_DIR}/config.h"
)

#
# Add source file
#
file(GLOB_RECURSE SOURCE_FILES src/*.cpp src/*.cu resources/*.cpp)
if(${LITES_CUDA_ON})
	list(REMOVE_ITEM SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/CMesh.cpp)
	message("CUDA On")
else(${LITES_CUDA_ON})
	list(REMOVE_ITEM SOURCE_FILES *.cu)
	message("CUDA Off")
endif(${LITES_CUDA_ON})

file(GLOB_RECURSE HEADER_FILES src/*.h resources/*.h "${PROJECT_BINARY_DIR}/config.h")

add_library (LiteS STATIC ${SOURCE_FILES} ${HEADER_FILES})
if(${LITES_CUDA_ON})
	set_target_properties(LiteS PROPERTIES POSITION_INDEPENDENT_CODE ON)
	set_target_properties(LiteS PROPERTIES RESOLVE_DEVICE_SYMBOLS ON)
	set_target_properties(LiteS PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
	set_target_properties(LiteS PROPERTIES CUDA_STANDARD 11)
endif(${LITES_CUDA_ON})
