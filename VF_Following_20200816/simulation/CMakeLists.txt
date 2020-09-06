# requirement of cmake version，指定cmake版本 
cmake_minimum_required(VERSION 3.2)

# project name, 指定项目的名称，一般和项目的文件夹名称对应
project(FV_Planner)


# set environment variable, 设置环境变量

# add library, 添加库
add_library(lsimulation STATIC
            simulation.cpp
            simulation.hpp)

