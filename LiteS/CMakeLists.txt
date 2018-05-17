if($ENV{OPENGL}&&$ENV{ASSIMP}&&$ENV{GLM})
	message(FATAL_ERROR "Not defined environment variable:OPENGL or ASSIMP or GLM")  
endif()

include_directories ("$ENV{OPENGL}/include")
include_directories ("$ENV{GLM}")
include_directories ("$ENV{ASSIMP}/include")

#SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
SET(CMAKE_INSTALL_PREFIX ${PROJECT_BINARY_DIR}/lib)

set(CMAKE_CXX_FLAGS_RELEASE "/MD")
set(CMAKE_CXX_FLAGS_DEBUG "/MDd")

# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_LIB_SRCS 变量
aux_source_directory(./src DIR_LIB_SRCS)

# 生成链接库
add_library (LiteS ${DIR_LIB_SRCS})

set_target_properties (LiteS PROPERTIES DEBUG_POSTFIX d)

IF(CMAKE_GENERATOR MATCHES .*Win64)
	INSTALL(TARGETS LiteS DESTINATION ${PROJECT_BINARY_DIR}/lib/x64)
ELSE()
	INSTALL(TARGETS LiteS DESTINATION ${PROJECT_BINARY_DIR}/lib/x86)
ENDIF(CMAKE_GENERATOR MATCHES .*Win64)

