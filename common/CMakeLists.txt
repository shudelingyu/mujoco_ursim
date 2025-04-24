# 设置最低的CMake版本要求
cmake_minimum_required(VERSION 3.10)

# 定义一个名为 'common' 的库
project(common)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MUJOCO_DIR "~/.mujoco/mujoco-3.2.7")  # 版本号需与实际安装一致
include_directories(${MUJOCO_DIR}/include)
link_directories(${MUJOCO_DIR}/bin)

find_package(OpenGL REQUIRED)
# find_package(glad REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)


# 设置公共头文件目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

# 如果有源文件，可以将它们添加到库中
add_library(common STATIC ${SOURCES}
)

target_link_libraries(common
    glfw
    ${OPENGL_LIBRARIES}
    GL
    GLEW
    Eigen3::Eigen
    ${MUJOCO_DIR}/lib/libmujoco.so.3.2.7
)

# 为这个库设置安装规则
install(TARGETS common DESTINATION lib)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/common/include/ DESTINATION include)

# 如果子项目需要使用此库，可以设置相关的链接选项
target_include_directories(common PUBLIC ${CMAKE_SOURCE_DIR}/common/include)
