SET(CaseName T-Robust)

if($ENV{OPENGL}&&$ENV{ASSIMP}&&$ENV{GLM})
	message(FATAL_ERROR "Not defined environment variable:OPENGL or ASSIMP or GLM")  
endif()

INCLUDE_DIRECTORIES ("$ENV{OPENGL}/include")
INCLUDE_DIRECTORIES ("$ENV{GLM}")
INCLUDE_DIRECTORIES ("$ENV{ASSIMP}/include")
INCLUDE_DIRECTORIES ("${PROJECT_SOURCE_DIR}/LiteS/src")
INCLUDE_DIRECTORIES ("$ENV{NVTX_PATH}/include")


IF(CMAKE_GENERATOR MATCHES .*Win64)
	LINK_DIRECTORIES("$ENV{OPENGL}/lib/x64")
	LINK_DIRECTORIES("$ENV{ASSIMP}/lib/x64")
	LINK_DIRECTORIES("$ENV{NVTX_PATH}/lib/x64")
	
	if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	# 设置Debug的库文件
		LINK_DIRECTORIES("${PROJECT_BINARY_DIR}/lib/x64")
	else(CMAKE_BUILD_TYPE STREQUAL "Release")
	# 设置Release的库文件
		LINK_DIRECTORIES("${PROJECT_BINARY_DIR}/lib/x64")
	endif()	
ELSE()
	LINK_DIRECTORIES("$ENV{OPENGL}/lib/x86")
	LINK_DIRECTORIES("$ENV{ASSIMP}/lib/x86")
	LINK_DIRECTORIES("$ENV{NVTX_PATH}/lib/win32")	
	if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	# 设置Debug的库文件
		LINK_DIRECTORIES("${PROJECT_BINARY_DIR}/lib/x86")
	else(CMAKE_BUILD_TYPE STREQUAL "Release")
	# 设置Release的库文件
		LINK_DIRECTORIES("${PROJECT_BINARY_DIR}/lib/x86")
	endif()	
ENDIF(CMAKE_GENERATOR MATCHES .*Win64)

STRING(REGEX REPLACE ".*/(.*)" "\\1" CURRENT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR} ) 
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin/${CURRENT_FOLDER})

set(CMAKE_CXX_FLAGS_RELEASE "/MD")
set(CMAKE_CXX_FLAGS_DEBUG "/MDd")

# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_LIB_SRCS 变量
#aux_source_directory(./src DIR_LIB_SRCS)

file (GLOB headers "src/*.h")
file (GLOB sources "src/*.cpp")
file (GLOB shaders "src/shaders/*")

source_group("sources" FILES ${sources})
source_group("headers" FILES ${headers})
source_group("" FILES src/config.xml)
source_group("shaders" FILES ${shaders})

# 指定生成目标
add_executable(${CaseName} ${sources} ${headers} ${shaders} src/config.xml)

add_dependencies(${CaseName} LiteS)
TARGET_LINK_LIBRARIES(${CaseName} debug LiteSd.lib optimized LiteS.lib glew32.lib opengl32.lib glfw3-vc140.lib assimp-vc140-mt.lib nvToolsExt32_1.lib)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	# 设置Debug的库文件
	INSTALL(FILES src/config.xml DESTINATION ${EXECUTABLE_OUTPUT_PATH}/Debug)
	INSTALL(DIRECTORY src/shaders DESTINATION ${EXECUTABLE_OUTPUT_PATH}/Debug)
	set_target_properties(${CaseName} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/Debug)
	
	ADD_DEFINITIONS( /Zi )
else(CMAKE_BUILD_TYPE STREQUAL "Release")
	# 设置Release的库文件
	INSTALL(FILES src/config.xml DESTINATION ${EXECUTABLE_OUTPUT_PATH}/Release)
	INSTALL(DIRECTORY src/shaders DESTINATION ${EXECUTABLE_OUTPUT_PATH}/Release)	
	set_target_properties(${CaseName} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/Release)
	
endif()	