

cmake_minimum_required(VERSION 3.12)
project(test_ik VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MuJoCo路径配置
include_directories(${MUJOCO_DIR}/include)
link_directories(${MUJOCO_DIR}/bin)

# 第三方库配置
# find_package(OpenGL REQUIRED)
# find_package(glfw3 REQUIRED)
# find_package(Eigen3 REQUIRED)

# 构建目标
add_executable(test_ik 
    src/main.cpp
)

set_target_properties(test_ik PROPERTIES DEBUG_POSTFIX d)

# 设置包含目录，如果需要的话
target_include_directories(test_ik PRIVATE ${CMAKE_SOURCE_DIR}/common/include)

# 引入 common 库
target_link_libraries(test_ik common)

if(WIN32)
    find_package(OpenGL REQUIRED)
    # find_package(glad REQUIRED)
    include_directories(${glfw_ROOT}/include)
    # find_package(GLEW REQUIRED)
    include_directories(${GLEW_ROOT}/include)
    find_package(Eigen3 REQUIRED NO_MODULE)
    include_directories(${qpOASES_ROOT}/include)
elseif(UNIX)
    find_package(OpenGL REQUIRED)
    # find_package(glad REQUIRED)
    find_package(glfw3 REQUIRED)
    find_package(Eigen3 REQUIRED)
    find_package(qpOASES REQUIRED)
endif()


if(WIN32)
    # 编译完成后自动复制 DLL
    add_custom_command(TARGET test_ik POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
            "${MUJOCO_DIR}/bin/mujoco.dll"
            "$<TARGET_FILE_DIR:test_ik>"
        COMMENT "Copying mujoco.dll to output directory"
    )
elseif(UNIX)
    set(CMAKE_EXE_LINKER_FLAGS "-Wl,-rpath,${MUJOCO_DIR}/bin")
endif()
# 资源文件处理
#file(COPY models DESTINATION ${CMAKE_BINARY_DIR})
