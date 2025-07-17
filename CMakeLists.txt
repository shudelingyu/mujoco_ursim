

cmake_minimum_required(VERSION 3.12)
project(mujoco_ursim)

# 2. 平台检测与路径区分
if(WIN32)

    # set(CMAKE_PREFIX_PATH "E:/mujoco_develop")
    set(MUJOCO_DIR "F:/mujoco-3.3.3-windows-x86_64")  # Windows路径
    set(devEnvPath "E:/mujoco_develop" CACHE PATH "packages base directory." FORCE)
		message("devEnvPath: use default value ${devEnvPath}")
    set(PAK_BASE "${devEnvPath}" CACHE PATH "packages base directory." FORCE)

    set(Eigen3_ROOT ${PAK_BASE}/Eigen3-3.4.0-msvc)
    set(GLEW_ROOT ${PAK_BASE}/glew-2.2.0-msvc)
    set(glfw_ROOT ${PAK_BASE}/glfw-3.4.bin.WIN64)
    set(qpOASES_ROOT ${PAK_BASE}/qpOASES)
elseif(UNIX)
    set(MUJOCO_DIR "$ENV{HOME}/.mujoco/mujoco-3.2.7")  # Linux/Mac路径
    set(qpEnvPath "/usr/local/")
endif()
include_directories(${MUJOCO_DIR}/include)
link_directories(${MUJOCO_DIR}/bin)

add_subdirectory(common)
add_subdirectory(ACSP)

# 添加子项目
# add_subdirectory(test1)
add_subdirectory(test_ik)
add_subdirectory(cartesian_admittance)
# add_subdirectory(test_Cartesian_impdance)
# add_subdirectory(test_Cartesian-Trajectory)
# add_subdirectory(test_DragInCST)
# add_subdirectory(testik_fk)
# add_subdirectory(test_Jacobian) 
# add_subdirectory(test_joint_impdance)
# add_subdirectory(testp5)



