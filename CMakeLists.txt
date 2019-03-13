cmake_minimum_required (VERSION 3.1)
project (LiteS)

function(ComplerOption)
    # Compiler and Linker options
    # Platform independent
    if(${CMAKE_GENERATOR} MATCHES ".*Visual Studio.*|.*Ninja.*")
        set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} /MD /Zi /GL /Od")
        set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} /MD")

        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /Profile /LTCG")
    elseif(${CMAKE_GENERATOR} MATCHES ".*Unix.*")
        SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
        SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
    else()
        message(FATAL_ERROR "Unsurpported generator")
    endif(${CMAKE_GENERATOR} MATCHES ".*Visual Studio.*|.*Ninja.*")
endfunction()

ComplerOption()

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/bin)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/bin)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/lib)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/lib)
SET(CMAKE_DEBUG_POSTFIX "_d")

ADD_SUBDIRECTORY(LiteS-Engine)
ADD_SUBDIRECTORY(LiteS-TestCase)