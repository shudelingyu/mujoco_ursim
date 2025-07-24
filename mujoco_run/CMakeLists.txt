cmake_minimum_required(VERSION 3.15)  # 升级以支持现代特性
project(mujoco_run VERSION 1.0 LANGUAGES C CXX)

# 核心配置
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # 禁用编译器扩展

# 确保在 Windows 上正确处理大目标文件
if(MSVC)
    add_compile_options(/bigobj)
endif()

target_include_directories(common
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

file(GLOB SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp  # 更结构化
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h  # 更结构化
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

# 可执行目标定义
add_executable(mujoco_run  ${SOURCES})

# 设置目标属性
set_target_properties(mujoco_run PROPERTIES
    DEBUG_POSTFIX d
    CXX_VISIBILITY_PRESET hidden  # 提升性能
    VISIBILITY_INLINES_HIDDEN ON
)

# 包含目录设置（使用目标特定指令）
target_include_directories(mujoco_run PRIVATE
    ${CMAKE_SOURCE_DIR}/common/include
    ${CMAKE_SOURCE_DIR}/algorithm/include
)

# MuJoCo 路径配置（推荐使用目标链接指令）
target_link_directories(mujoco_run PRIVATE ${MUJOCO_DIR}/bin)

# 第三方库查找 - 使用现代 find_package 方式
find_package(OpenGL REQUIRED)
find_package(Eigen3 REQUIRED CONFIG)  # 使用 CONFIG 模式

# 平台特定配置
if(WIN32)
    # 使用现代路径设置方式
    find_package(glfw3 CONFIG)  # 使用 CONFIG 模式
    
    # GLEW 配置
    if(GLEW_ROOT)
        target_include_directories(mujoco_run PRIVATE ${GLEW_ROOT}/include)
    endif()
    
    # qpOASES 配置
    if(qpOASES_ROOT)
        target_include_directories(mujoco_run PRIVATE ${qpOASES_ROOT}/include)
    endif()
    
    # 编译后复制 DLL
    add_custom_command(TARGET mujoco_run POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
            "${MUJOCO_DIR}/bin/mujoco.dll"
            "$<TARGET_FILE_DIR:mujoco_run>"
        COMMENT "复制 mujoco.dll 到输出目录"
    )
elseif(UNIX)
    # 使用现代 find_package
    find_package(glfw3 CONFIG REQUIRED)
    find_package(GLEW REQUIRED)
    
    # 设置 rpath 确保运行时能找到 MuJoCo
    set_target_properties(mujoco_run PROPERTIES
        INSTALL_RPATH "${MUJOCO_DIR}/lib"
        BUILD_WITH_INSTALL_RPATH TRUE
    )
endif()

# 库链接 - 使用现代 CMake 目标
target_link_libraries(mujoco_run PRIVATE
    common
    algorithm
    Eigen3::Eigen
    OpenGL::OpenGL  # 使用现代 OpenGL 目标
)

# 平台特定库链接
if(WIN32)
    target_link_libraries(mujoco_run PRIVATE
        ${glfw_ROOT}/lib-vc2019/glfw3.lib
        ${GLEW_ROOT}/lib/glew32rs.lib
        ${qpOASES_ROOT}/lib/qpOASES.lib
        ${MUJOCO_DIR}/lib/mujoco.lib
    )
elseif(UNIX)
    target_link_libraries(mujoco_run PRIVATE
        glfw
        GLEW::GLEW
        ${qpEnvPath}/lib/libqpOASES.a
        ${MUJOCO_DIR}/lib/libmujoco.so.3.2.7
    )
endif()

# 资源文件处理（如果需要）
# 使用更健壮的方法复制资源文件
if(EXISTS ${CMAKE_SOURCE_DIR}/models)
    add_custom_command(TARGET mujoco_run POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/models
            $<TARGET_FILE_DIR:mujoco_run>/models
        COMMENT "复制模型文件到输出目录"
    )
endif()

# 安装规则（如果项目需要安装）
install(TARGETS mujoco_run
    RUNTIME DESTINATION bin
    BUNDLE DESTINATION bin
)

# 添加测试支持（可选）
enable_testing()
add_test(NAME basic_test COMMAND mujoco_run --test)