project (LiteS-CUDA)

find_package(CUDA)

file(GLOB_RECURSE SOURCE_FILES ./*.cpp ./*.cu )
file(GLOB_RECURSE HEADER_FILES ./*.h)

include_directories (../LiteS-Engine/src/)

add_library(LiteS-CUDA STATIC ${SOURCE_FILES} ${HEADER_FILES})

add_dependencies(LiteS-CUDA LiteS)
set_target_properties(LiteS-CUDA PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
