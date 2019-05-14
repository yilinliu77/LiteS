project (LiteS-CUDA LANGUAGES CUDA CXX)

enable_language(CUDA)

# set this in CMakeLists.txt in root dir, because it only works on windows
# set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler \"/wd 4819\"")
set(CMAKE_CUDA_STANDARD 11)
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_CUDA_RESOLVE_DEVICE_SYMBOLS ON)
set(CUDA_SEPARABLE_COMPILATION ON)

file(GLOB_RECURSE SOURCE_FILES ./*.cpp ./*.cu )
file(GLOB_RECURSE HEADER_FILES ./*.h)

include_directories (../LiteS-Engine/src/)

add_library(LiteS-CUDA STATIC ${SOURCE_FILES} ${HEADER_FILES})

add_dependencies(LiteS-CUDA LiteS)
