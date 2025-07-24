# 设置最低的CMake版本要求
cmake_minimum_required(VERSION 3.10)

# 定义一个名为 'common' 的库
project(common)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(ACSP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../ACSP)


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
    include_directories(${qpEnvPath}/include)
endif()



# 设置公共头文件目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
# include_directories(${ACSP_DIR}/src ${ACSP_DIR}/FastMath)
file(GLOB SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

# 如果有源文件，可以将它们添加到库中
add_library(common STATIC ${SOURCES} 
)

if (MSVC)
    target_compile_options(common PRIVATE /bigobj)
endif()

if(WIN32)
    target_link_libraries(common
        PUBLIC ${glfw_ROOT}/lib-vc2019/glfw3.lib
        PUBLIC ${GLEW_ROOT}/lib/glew32rs.lib
        PUBLIC opengl32.lib   # Windows OpenGL库
        #PUBLIC GLEW::glew
        PUBLIC Eigen3::Eigen
        PUBLIC ${qpOASES_ROOT}/lib/qpOASES.lib
        PUBLIC ${MUJOCO_DIR}/lib/mujoco.lib  # Windows静态库
    )
elseif(UNIX)
    target_link_libraries(common 
        glfw
        ${OPENGL_LIBRARIES} 
        GL
        GLEW
        Eigen3::Eigen
        ${qpEnvPath}/lib/libqpOASES.a
        ${MUJOCO_DIR}/lib/libmujoco.so.3.2.7
    )
endif()

# 8. 跨平台安装规则
install(TARGETS common
    ARCHIVE DESTINATION lib   # 静态库 (.a/.lib)
    LIBRARY DESTINATION lib   # 动态库 (.so/.dll)
    RUNTIME DESTINATION bin   # 可执行文件 (Windows DLL)
)
install(DIRECTORY include/ DESTINATION include)  # 头文件安装

# 如果子项目需要使用此库，可以设置相关的链接选项
target_include_directories(common PUBLIC ${CMAKE_SOURCE_DIR}/common/include)