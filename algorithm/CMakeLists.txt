cmake_minimum_required(VERSION 3.15)  # 升级到更现代的版本

# 定义项目
project(algorithm VERSION 1.0.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # 禁用编译器扩展

# 使用现代目标模式替代全局设置
add_library(algorithm STATIC)

# 配置公共头文件目录
target_include_directories(algorithm
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/algorithm>
        $<INSTALL_INTERFACE:include>
)

# 添加源文件（明确列出比GLOB更好，但保留GLOB作为示例）
file(GLOB SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/algorithm/*.hpp  # 更结构化
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)
target_sources(algorithm PRIVATE ${SOURCES})

# 平台特定依赖配置
if(WIN32)
    find_package(OpenGL REQUIRED)
    find_package(Eigen3 REQUIRED CONFIG)  # 使用CONFIG模式
    
    # 使用现代目标链接方式
    target_include_directories(algorithm
        PRIVATE
            ${glfw_ROOT}/include
            ${GLEW_ROOT}/include
            ${qpOASES_ROOT}/include
    )
    
    target_link_libraries(algorithm PRIVATE
        ${glfw_ROOT}/lib-vc2019/glfw3.lib
        ${GLEW_ROOT}/lib/glew32rs.lib
        OpenGL::GL
        Eigen3::Eigen
        ${qpOASES_ROOT}/lib/qpOASES.lib
        ${MUJOCO_DIR}/lib/mujoco.lib
    )
    
    if(MSVC)
        target_compile_options(algorithm PRIVATE /bigobj)
    endif()

elseif(UNIX)
    find_package(Eigen3 REQUIRED CONFIG)
    
    target_link_libraries(algorithm PRIVATE
        Eigen3::Eigen
        ${qpEnvPath}/lib/libqpOASES.a
    )
    
    target_include_directories(algorithm PRIVATE
        ${qpEnvPath}/include
    )
endif()

# 安装规则（现代CMake风格）
install(TARGETS algorithm
    EXPORT algorithmTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(
    DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# 导出目标（供其他项目使用）
# install(EXPORT commonTargets
#     FILE commonTargets.cmake
#     NAMESPACE common::
#     DESTINATION lib/cmake/common
# )

# # 创建配置文件
# include(CMakePackageConfigHelpers)
# # configure_package_config_file(
# #     # ${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
# #     ${CMAKE_CURRENT_BINARY_DIR}/commonConfig.cmake
# #     INSTALL_DESTINATION lib/cmake/common
# # )

# write_basic_package_version_file(
#     ${CMAKE_CURRENT_BINARY_DIR}/commonConfigVersion.cmake
#     VERSION ${PROJECT_VERSION}
#     COMPATIBILITY SameMajorVersion
# )

# install(FILES
#     ${CMAKE_CURRENT_BINARY_DIR}/commonConfig.cmake
#     ${CMAKE_CURRENT_BINARY_DIR}/commonConfigVersion.cmake
#     DESTINATION lib/cmake/common
# )