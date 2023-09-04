cmake_minimum_required(VERSION 3.15)
# 设置工程名
project(pure VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)
add_executable(${PROJECT_NAME} )

# 设置目标的编译参数，讲警告等级设置为最高
#target_compile_options(${PROJECT_NAME} PRIVATE
#        -Wall -Wextra            # 尽可能多的错误
#        -Wno-unknown-pragmas     # 无效的pragmas
#        -Werror -Wfatal-errors   # 将所有的警告视为错误并立即停止编译
#        -Wfloat-equal            # 浮点数对比
#        -ffunction-sections      # 用于section优化
#        -Wsign-compare           # 有符号与无符号比较
#        -Wdisabled-optimization  # 不能被优化
#        -Wshadow                 # 当一个局部变量遮盖住了另一个局部变量
#        -Wnarrowing              # 窄转换
#        -Woverloaded-virtual     # 如果函数的声明隐藏住了基类的虚函数
#        -Wdouble-promotion       # 禁止隐形float提升为double
#        -Wno-return-local-addr   # 返回局部地址
#        -Wswitch                 # switch覆盖
#        )

# 统计当前文件下所有参与编译的文件
file(GLOB_RECURSE SRC_LIST
        CONFIGURE_DEPENDS
        src/*.cpp
        src/*.c
        src/*.cxx
        src/*.h
        src/*.hpp)

# 将文件加入到工程依赖
target_sources(${PROJECT_NAME} PRIVATE ${SRC_LIST})


#Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#Python
find_package(Python3 COMPONENTS Development NumPy)
target_link_libraries(${PROJECT_NAME} Python3::Python Python3::NumPy)
