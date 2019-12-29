project (LiteS-CUDA)

enable_language(CUDA)
include(CheckLanguage)
check_language(CUDA)

# set this in CMakeLists.txt in root dir, because it only works on windows
#set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS}")

file(GLOB_RECURSE SOURCE_FILES ./*.cpp ./*.cu )
file(GLOB_RECURSE HEADER_FILES ./*.h)

add_library(LiteS-CUDA STATIC ${SOURCE_FILES} ${HEADER_FILES})
set_target_properties(LiteS-CUDA PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(LiteS-CUDA PROPERTIES RESOLVE_DEVICE_SYMBOLS ON)
set_target_properties(LiteS-CUDA PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties(LiteS-CUDA PROPERTIES CUDA_STANDARD 11)

add_dependencies(LiteS-CUDA LiteS)
