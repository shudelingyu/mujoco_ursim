

cmake_minimum_required(VERSION 3.12)
project(cartesian_admittance VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MuJoCo路径配置
set(MUJOCO_DIR "~/.mujoco/mujoco-3.2.7")  # 版本号需与实际安装一致
include_directories(${MUJOCO_DIR}/include)
link_directories(${MUJOCO_DIR}/bin)

# 第三方库配置
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# 构建目标
add_executable(cartesian_admittance 
    src/main.cpp
)

set_target_properties(cartesian_admittance PROPERTIES DEBUG_POSTFIX d)

# 设置包含目录，如果需要的话
target_include_directories(cartesian_admittance PRIVATE ${CMAKE_SOURCE_DIR}/common/include)

# 引入 common 库
target_link_libraries(cartesian_admittance common)

target_link_libraries(cartesian_admittance
    glfw
    ${OPENGL_LIBRARIES}
    GL
    GLEW
)


set(CMAKE_EXE_LINKER_FLAGS "-Wl,-rpath,${MUJOCO_DIR}/bin")
# 资源文件处理
#file(COPY models DESTINATION ${CMAKE_BINARY_DIR})

