#!/bin/bash

function RemoveFolder {
    CURRENT_DIR=$(pwd)
    echo "Current path is: $CURRENT_DIR"
    cd "$CURRENT_DIR/.."
    #检查build &件夹是否存在
    if [ -d "build" ]; then
        echo "Deleting existing build directory..."
        rm -r build
    fi
}

function CreateFolder {
    #创建新的build文件夹
    mkdir build
}

function Compile {
    #进入build文件夹 
    cd build
    #运行CMake构建
    echo "Running CMake..."
    cmake ..
    #编译项目
    make -j20
    echo "------Build completed-----" 
}

function Format {
    CURRENT_DIR=$(pwd)
    echo "Current path is: $CURRENT_DIR"
    cd "$CURRENT_DIR/.."
    find . -name "*.cpp" -or -name "*.h" -or -name "*.hpp" | xargs clang-format -i -style=file  #整个项目格式化
}


#后缓选择 
case $1 in 
rm)
    RemoveFolder
    ;;
format) 
    Format
    ;;
compile)
    # Format
    RemoveFolder
    CreateFolder 
    Compile
    ;;
make)
    Format
    cd build
    make -j20
    cd ../shell_script
    ;;
*)
;;
esac
