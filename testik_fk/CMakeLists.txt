

cmake_minimum_required(VERSION 3.12)
project(testik_fk VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MuJoCo路径配置
set(MUJOCO_DIR "~/.mujoco/mujoco-3.2.7")  # 版本号需与实际安装一致
include_directories(${MUJOCO_DIR}/include)
link_directories(${MUJOCO_DIR}/bin)

# 第三方库配置
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)

# 构建目标
add_executable(testik_fk 

    src/modern_robotics.hpp
    src/modern_robotics.cpp
    src/zj_mujoco.hpp
    src/main.cpp
)

set_target_properties(testik_fk PROPERTIES DEBUG_POSTFIX d)

target_link_libraries(testik_fk
    glfw
    ${OPENGL_LIBRARIES}
    GL
    GLEW
    Eigen3::Eigen
    ${MUJOCO_DIR}/lib/libmujoco.so.3.2.7
)


set(CMAKE_EXE_LINKER_FLAGS "-Wl,-rpath,${MUJOCO_DIR}/bin")
# 资源文件处理
#file(COPY models DESTINATION ${CMAKE_BINARY_DIR})

