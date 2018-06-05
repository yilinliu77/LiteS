# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.1)
# 项目信息
project (LiteS)

SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# 添加 math 子目录
ADD_SUBDIRECTORY(LiteS)

ADD_SUBDIRECTORY(LiteS-TestCase)