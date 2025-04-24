#!/bin/bash

# 设置项目根目录
PROJECT_DIR=$(pwd)

# 创建并进入构建目录
BUILD_DIR="$PROJECT_DIR/build"
if [ ! -d "$BUILD_DIR" ]; then
  echo "创建构建目录: $BUILD_DIR"
  mkdir "$BUILD_DIR"
fi
cd "$BUILD_DIR"

# 运行 CMake 配置
echo "正在运行 CMake 配置..."
cmake ..

# 检查 CMake 配置是否成功
if [ $? -ne 0 ]; then
  echo "CMake 配置失败"
  exit 1
fi

# 执行 make 构建
echo "正在执行 make 构建..."
make

# 检查 make 构建是否成功
if [ $? -ne 0 ]; then
  echo "构建失败"
  exit 1
fi

echo "构建成功!"

