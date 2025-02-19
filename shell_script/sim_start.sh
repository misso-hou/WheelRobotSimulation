#!/bin/bash

# 设置可执行文件名
EXECUTABLE="./simulator"
CSV="./csvplot"

CURRENT_DIR=$(pwd)

echo "Current path is: $CURRENT_DIR"
cd "$CURRENT_DIR/../build/bin"

function StartSimulator {
    # 检查可执行文件是否存在
    if [ -f "$EXECUTABLE" ]; then
        # 运行可执行文件
        $EXECUTABLE
    else
        echo "Error: $EXECUTABLE 不存在或不可执行."
        exit 1
    fi
}

function StartCsvReader {
    cd csvPlt
    # 检查可执行文件是否存在
    if [ -f "$CSV" ]; then
        # 运行可执行文件
        $CSV
    else
        echo "Error: $CSV 不存在或不可执行."
        exit 1
    fi
}

case $1 in
csv)
    StartCsvReader
  ;;
*)
    StartSimulator
;;
esac