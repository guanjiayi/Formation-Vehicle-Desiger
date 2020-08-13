# requirement of cmake version，指定cmake版本 
cmake_minimum_required(VERSION 3.2)

# project name, 指定项目的名称，一般和项目的文件夹名称对应
project(FV_Planner)

# head file path, 头文件目录
#include_directories(canmessage/ 
#                    control/ 
#                    decision/ 
#                    planning/
#                    main/
#)

include_directories(${FV_Following_SOURCE_DIR}/canmessage 
                    ${FV_Following_SOURCE_DIR}/control 
                    ${FV_Following_SOURCE_DIR}/decider 
                    ${FV_Following_SOURCE_DIR}/planning                   
)


#add_subdirectory(include) 

add_subdirectory(canmessage) 
add_subdirectory(decision) 
add_subdirectory(control) 
add_subdirectory(planning) 
# add_subdirectory(main) 


# source directory, 源文件目录
# AUX_SOURCE_DIRECTORY(./main DIR_SRCS)
AUX_SOURCE_DIRECTORY(./main SRC_LIST)

MESSAGE(${SRC_LIST})

# set environment variable, 设置环境变量，编译用的源文件全部要放到这里，否则编译能通过，但是执行的时候会出现各种问题，比如"symbol lookup error xxxxx , undefined symbol"
# SET(TEST_MATH ${DIR_SRCS})

# set(CMAKE_CXX_STANDARD 11)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else() 
     message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()



# add executable file，添加要编译的可执行文件
# ADD_EXECUTABLE(${PROJECT_NAME} ${TEST_MATH})
ADD_EXECUTABLE(FV_Planner ${SRC_LIST})

# Find thread package
find_package(Threads REQUIRED)

# add link library, 添加可执行文件所需要的库，比如我们用到的libm.so（命名规则：lib+name+.so），就添加该库的名称
# TARGET_LINK_LIBRARIES(${PROJECT_NAME} m)

target_link_libraries(FV_Planner
                      # Threads::Threads 
                      lcanmessage 
                      lcontrol 
                      ldecision 
                      lplanning)

