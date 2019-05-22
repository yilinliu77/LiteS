project (LiteS-CUDA)

include_directories("C:/repos/Graphics/RENDERING/LiteS/LiteS-Engine/src/")


enable_language(CUDA)

# set this in CMakeLists.txt in root dir, because it only works on windows
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS}")
set(CMAKE_CUDA_STANDARD 11)
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_RESOLVE_DEVICE_SYMBOLS ON)
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)

file(GLOB_RECURSE SOURCE_FILES ./*.cpp ./*.cu )
file(GLOB_RECURSE HEADER_FILES ./*.h)

add_library(LiteS-CUDA STATIC ${SOURCE_FILES} ${HEADER_FILES})
set_target_properties(LiteS-CUDA PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_dependencies(LiteS-CUDA LiteS)
