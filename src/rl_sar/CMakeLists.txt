cmake_minimum_required(VERSION 3.10)
project(rl_sar VERSION 4.0.0 LANGUAGES CXX)

# Set C++ standard globally
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# System environment detection
set(SYSTEM_TYPE ${CMAKE_SYSTEM_NAME})

set(USE_CMAKE OFF CACHE BOOL "Use Cmake build system")
if(USE_CMAKE)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        add_compile_definitions(USE_ROS1)
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        add_compile_definitions(USE_ROS2)
    endif()
else()
    message(WARNING "ROS_DISTRO not set, assuming non-ROS mode")
    add_compile_definitions(USE_CMAKE)
endif()

# MuJoCo simulator option
set(USE_MUJOCO OFF CACHE BOOL "Build with MuJoCo simulator support")

# ZeroMQ link to the OCS2 MPC bridge (go2_x5_ocs2_bridge). Off by default so the
# stock build needs no libzmq; the OCS2 FSM state is compiled out with it.
set(USE_OCS2_BRIDGE OFF CACHE BOOL "Build with the OCS2 MPC ZeroMQ bridge")

# JoyLink gamepad client (https://github.com/ -- local checkout at JOYLINK_ROOT).
# MuJoCo-sim-only alternate to raw /dev/input/js0 reading: a `joylink` server
# (built separately from JOYLINK_ROOT) normalizes a specific pad's axes/buttons
# once via YAML instead of guessing indices/polarity in C++. Off by default so
# the stock build needs neither libzmq nor a JoyLink checkout.
set(USE_JOYLINK OFF CACHE BOOL "Build rl_sim_mujoco with JoyLink gamepad client support")
set(JOYLINK_ROOT "" CACHE PATH "Path to a built JoyLink checkout (must already contain build/client, build/common, build/_deps/yaml-cpp-build)")

# The bundled Agibot D1 binary requires glibc 2.34. Keep the rest of rl_sar
# buildable on Ubuntu 20.04 / JetPack systems that provide an older glibc.
option(BUILD_AGIBOT_D1 "Build the Agibot D1 real-robot executable" ON)
if(SYSTEM_TYPE STREQUAL "Linux" AND BUILD_AGIBOT_D1)
    execute_process(
        COMMAND getconf GNU_LIBC_VERSION
        OUTPUT_VARIABLE HOST_GLIBC_VERSION_OUTPUT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    string(REGEX MATCH "[0-9]+\\.[0-9]+" HOST_GLIBC_VERSION "${HOST_GLIBC_VERSION_OUTPUT}")
    if(HOST_GLIBC_VERSION AND HOST_GLIBC_VERSION VERSION_LESS "2.34")
        message(WARNING
            "Disabling rl_real_d1: bundled Agibot D1 SDK requires glibc >= 2.34, "
            "but the host provides ${HOST_GLIBC_VERSION}")
        set(BUILD_AGIBOT_D1 OFF)
    endif()
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" UPPER_BUILD_TYPE)
if(UPPER_BUILD_TYPE STREQUAL "DEBUG")
    add_compile_options(-g)
endif()

# Get project root directory
get_filename_component(PROJECT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPOLICY_DIR="${PROJECT_ROOT_DIR}/policy")
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        find_package(catkin REQUIRED COMPONENTS
            controller_manager
            genmsg
            joint_state_controller
            robot_state_publisher
            roscpp
            gazebo_ros
            std_msgs
            tf
            geometry_msgs
            robot_msgs
            robot_joint_controller
            rospy
        )
        catkin_package(
            CATKIN_DEPENDS
            robot_joint_controller
            rospy
        )
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        find_package(ament_cmake REQUIRED)
        find_package(joint_state_broadcaster REQUIRED)
        find_package(robot_state_publisher REQUIRED)
        find_package(rclcpp REQUIRED)
        find_package(gazebo_ros REQUIRED)
        find_package(std_msgs REQUIRED)
        find_package(robot_msgs REQUIRED)
        find_package(robot_joint_controller REQUIRED)
        find_package(rclpy REQUIRED)
        find_package(gazebo_msgs REQUIRED)
        find_package(std_srvs REQUIRED)
        find_package(geometry_msgs REQUIRED)
        find_package(nav_msgs REQUIRED)
        find_package(unitree_api REQUIRED)
        find_package(unitree_go REQUIRED)
        ament_package()
    endif()
    add_compile_options(${GAZEBO_CXX_FLAGS})
    find_package(gazebo REQUIRED)
endif()

# Inference Runtime

set(INFERENCE_RUNTIME_DIR "${PROJECT_ROOT_DIR}/library/inference_runtime")

# ONNX Runtime Setup
set(USE_ONNX "OFF")
set(ONNX_RUNTIME_LIB "")
set(ONNX_RUNTIME_DIR "${INFERENCE_RUNTIME_DIR}/onnxruntime")

# Detect Jetson platform
set(IS_JETSON FALSE)
if(EXISTS "/etc/nv_tegra_release")
    set(IS_JETSON TRUE)
    message(STATUS "Jetson platform detected - ONNX Runtime will be disabled")
endif()

if(EXISTS "${ONNX_RUNTIME_DIR}/include" AND EXISTS "${ONNX_RUNTIME_DIR}/lib" AND NOT IS_JETSON)
    set(USE_ONNX "ON")
    add_compile_definitions(USE_ONNX)

    message(STATUS "Found ONNX Runtime at: ${ONNX_RUNTIME_DIR}")

    # Read ONNX Runtime version
    if(EXISTS "${ONNX_RUNTIME_DIR}/VERSION_NUMBER")
        file(READ "${ONNX_RUNTIME_DIR}/VERSION_NUMBER" ONNX_RUNTIME_VERSION)
        string(STRIP "${ONNX_RUNTIME_VERSION}" ONNX_RUNTIME_VERSION)
    endif()

    include_directories("${ONNX_RUNTIME_DIR}/include")

    # Detect library extension
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(LIB_EXT "so")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set(LIB_EXT "dylib")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        set(LIB_EXT "lib")
    endif()

    # Find library
    if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        set(LIB_NAMES onnxruntime.lib)
    else()
        set(LIB_NAMES libonnxruntime.${LIB_EXT} onnxruntime.${LIB_EXT})
    endif()

    find_library(ONNX_RUNTIME_LIB_PATH
        NAMES ${LIB_NAMES}
        PATHS "${ONNX_RUNTIME_DIR}/lib"
        NO_DEFAULT_PATH
    )

    if(ONNX_RUNTIME_LIB_PATH)
        if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
            add_library(onnxruntime_prebuilt STATIC IMPORTED GLOBAL)
        else()
            add_library(onnxruntime_prebuilt SHARED IMPORTED GLOBAL)
        endif()

        set_target_properties(onnxruntime_prebuilt PROPERTIES
            IMPORTED_LOCATION "${ONNX_RUNTIME_LIB_PATH}"
            INTERFACE_INCLUDE_DIRECTORIES "${ONNX_RUNTIME_DIR}/include"
        )

        set(ONNX_RUNTIME_LIB onnxruntime_prebuilt)
        message(STATUS "ONNX Runtime library: ${ONNX_RUNTIME_LIB_PATH}")
    else()
        message(WARNING "ONNX Runtime library not found")
        set(USE_ONNX "OFF")
    endif()
else()
    message(WARNING "ONNX Runtime not found. Please run: bash download_inference_runtime.sh onnx")
endif()

# LibTorch Setup
set(USE_TORCH "OFF")
set(TORCH_LIBRARIES "")
set(LIBTORCH_DIR "${INFERENCE_RUNTIME_DIR}/libtorch")

if(EXISTS "${LIBTORCH_DIR}/include" AND EXISTS "${LIBTORCH_DIR}/lib")
    set(USE_TORCH "ON")
    add_compile_definitions(USE_TORCH)

    message(STATUS "Found LibTorch at: ${LIBTORCH_DIR}")

    # For Jetson with old CMake: set common Jetson CUDA architectures to avoid CUDA17 requirement
    if(CMAKE_VERSION VERSION_LESS "3.18")
        # Common Jetson architectures: 5.3(Nano), 6.2(TX2), 7.2(Xavier), 8.7(Orin)
        set(TORCH_CUDA_ARCH_LIST "5.3;6.2;7.2;8.7" CACHE STRING "CUDA architectures")
        set(ENV{TORCH_CUDA_ARCH_LIST} "5.3;6.2;7.2;8.7")
        message(STATUS "CMake ${CMAKE_VERSION} detected - using common Jetson CUDA architectures")
    endif()

    set(CMAKE_PREFIX_PATH "${LIBTORCH_DIR};${CMAKE_PREFIX_PATH}")
    find_package(Torch QUIET)

    if(Torch_FOUND)
        message(STATUS "LibTorch version: ${Torch_VERSION}")
        message(STATUS "Torch libraries: ${TORCH_LIBRARIES}")
    else()
        set(USE_TORCH "OFF")
        message(WARNING "LibTorch found but find_package(Torch) failed")
    endif()
else()
    message(WARNING "LibTorch not found. Please run: bash download_inference_runtime.sh libtorch")
endif()

# RPATH Configuration for inference libraries
if(USE_TORCH OR USE_ONNX)
    set(CMAKE_SKIP_BUILD_RPATH FALSE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

    set(PREBUILT_LIB_PATHS "")
    if(USE_TORCH)
        list(APPEND PREBUILT_LIB_PATHS "${LIBTORCH_DIR}/lib")
    endif()
    if(USE_ONNX)
        list(APPEND PREBUILT_LIB_PATHS "${ONNX_RUNTIME_DIR}/lib")
    endif()
    # OpenMP path will be added later after it's found on macOS
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${PREBUILT_LIB_PATHS}")

    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")
    endif()

    message(STATUS "Configured RPATH:")
    foreach(path ${PREBUILT_LIB_PATHS})
        message(STATUS "  - ${path}")
    endforeach()
endif()

find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Add Python library path to RPATH
if(Python3_LIBRARIES)
    get_filename_component(PYTHON_LIB_DIR "${Python3_LIBRARIES}" DIRECTORY)
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${PYTHON_LIB_DIR}")
    message(STATUS "Added Python library path to RPATH: ${PYTHON_LIB_DIR}")
endif()

if(SYSTEM_TYPE STREQUAL "Linux")
    set(YAML_CPP_LIBRARIES yaml-cpp)
link_directories(/usr/local/lib)
include_directories(${YAML_CPP_INCLUDE_DIR})
elseif(SYSTEM_TYPE STREQUAL "Darwin")
    find_library(YAML_CPP_LIB yaml-cpp REQUIRED)
    set(YAML_CPP_LIBRARIES ${YAML_CPP_LIB})
    link_directories(/opt/homebrew/lib)
    if(YAML_CPP_INCLUDE_DIR)
        include_directories(${YAML_CPP_INCLUDE_DIR})
    endif()

    # Find OpenMP (required by LibTorch on macOS)
    find_library(OMP_LIB NAMES omp PATHS /opt/homebrew/lib /opt/homebrew/opt/libomp/lib)
    if(OMP_LIB)
        message(STATUS "Found OpenMP library: ${OMP_LIB}")
        set(OPENMP_LIBRARIES ${OMP_LIB})
        # Add OpenMP path to RPATH (for LibTorch dependency)
        get_filename_component(OMP_LIB_DIR "${OPENMP_LIBRARIES}" DIRECTORY)
        set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${OMP_LIB_DIR}")
        message(STATUS "Added OpenMP to RPATH: ${OMP_LIB_DIR}")
    else()
        message(WARNING "OpenMP not found. Please install: brew install libomp")
    endif()
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    set(ARCH_DIR "aarch64")
    set(UNITREE_LIB "libunitree_legged_sdk_arm64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    set(ARCH_DIR "x86_64")
    set(UNITREE_LIB "libunitree_legged_sdk_amd64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# unitree_legged_sdk
if(SYSTEM_TYPE STREQUAL "Linux")
add_library(unitree_legged_sdk SHARED IMPORTED)
set_target_properties(unitree_legged_sdk PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/unitree/unitree_legged_sdk/include"
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/unitree/unitree_legged_sdk/lib/${UNITREE_LIB}.so"
)
set(UNITREE_A1_LIBS Threads::Threads unitree_legged_sdk lcm)
if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
    file(GLOB GLOB_UNITREE_LEGGED_SDK "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/unitree/unitree_legged_sdk/lib/${UNITREE_LIB}.so")
    install(FILES
        ${GLOB_UNITREE_LEGGED_SDK}
        DESTINATION lib/
    )
    endif()
endif()

# unitree_sdk2
if(SYSTEM_TYPE STREQUAL "Linux")
    set(UNITREE_SDK2_ROOT
        ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/unitree/unitree_sdk2)
    set(UNITREE_SDK2_INCLUDE_DIRS
        ${UNITREE_SDK2_ROOT}/include
        ${UNITREE_SDK2_ROOT}/thirdparty/include
        ${UNITREE_SDK2_ROOT}/thirdparty/include/ddscxx
    )
    add_library(unitree_ddsc SHARED IMPORTED GLOBAL)
    set_target_properties(unitree_ddsc PROPERTIES
        IMPORTED_LOCATION ${UNITREE_SDK2_ROOT}/thirdparty/lib/${ARCH_DIR}/libddsc.so)
    add_library(unitree_ddscxx SHARED IMPORTED GLOBAL)
    set_target_properties(unitree_ddscxx PROPERTIES
        IMPORTED_LOCATION ${UNITREE_SDK2_ROOT}/thirdparty/lib/${ARCH_DIR}/libddscxx.so)
    target_link_libraries(unitree_ddscxx INTERFACE unitree_ddsc Threads::Threads)

    add_library(unitree_sdk2 STATIC IMPORTED GLOBAL)
    set_target_properties(unitree_sdk2 PROPERTIES
        IMPORTED_LOCATION ${UNITREE_SDK2_ROOT}/lib/${ARCH_DIR}/libunitree_sdk2.a)
    target_include_directories(unitree_sdk2 INTERFACE ${UNITREE_SDK2_INCLUDE_DIRS})
    target_link_libraries(unitree_sdk2 INTERFACE unitree_ddsc unitree_ddscxx Threads::Threads)
endif()

# l4w4_sdk
add_library(l4w4_sdk INTERFACE)
target_include_directories(l4w4_sdk INTERFACE
    library/thirdparty/robot_sdk/zhinao/l4w4_sdk
)
target_link_libraries(l4w4_sdk INTERFACE Threads::Threads)

# Lite3_MotionSDK
if(SYSTEM_TYPE STREQUAL "Linux")
add_library(lite3_motionsdk SHARED IMPORTED)
set_target_properties(lite3_motionsdk PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/deeprobotics/Lite3_MotionSDK/include;${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/deeprobotics/Lite3_MotionSDK/include/common"
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/deeprobotics/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_${ARCH_DIR}.so"
)
set(LITE3_REAL_LIBS Threads::Threads lite3_motionsdk)
if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
    file(GLOB GLOB_LITE3_SDK_SO "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/deeprobotics/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_${ARCH_DIR}.so")
    install(FILES
        ${GLOB_LITE3_SDK_SO}
        DESTINATION lib/
    )
    endif()
endif()

# Agibot D1 SDK (zsl-1)
if(SYSTEM_TYPE STREQUAL "Linux")
    add_library(agibot_d1_sdk SHARED IMPORTED)
    set_target_properties(agibot_d1_sdk PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/agibot/agibot_D1_Edu-Ultra/include"
        IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/agibot/agibot_D1_Edu-Ultra/lib/zsl-1/${ARCH_DIR}/libmc_sdk_zsl_1_${ARCH_DIR}.so"
    )
    set(AGIBOT_D1_LIBS Threads::Threads agibot_d1_sdk)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        file(GLOB GLOB_AGIBOT_D1_SDK_SO "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/agibot/agibot_D1_Edu-Ultra/lib/zsl-1/${ARCH_DIR}/libmc_sdk_zsl_1_${ARCH_DIR}.so")
        install(FILES
            ${GLOB_AGIBOT_D1_SDK_SO}
            DESTINATION lib/
        )
    endif()
endif()

# Retroid Gamepad
if(SYSTEM_TYPE STREQUAL "Linux")
set(GAMEPAD_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/deeprobotics/gamepad/src/gamepad.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/deeprobotics/gamepad/src/retroid_gamepad.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/deeprobotics/gamepad/src/udp_receiver.cpp
)
endif()

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        include_directories(${catkin_INCLUDE_DIRS})
    endif()
endif()

include_directories(
    include
    library/core/matplotlibcpp
    library/core/observation_buffer
    library/core/rl_sdk
    library/core/loop
    library/core/fsm
    library/core/vector_math
    library/core/motion_loader
    library/core/inference_runtime
    library/core/logger
    library/core/ocs2_bridge
    fsm_robot
)

# library for the OCS2 MPC bridge
if(USE_OCS2_BRIDGE)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(ZMQ REQUIRED libzmq)

    add_library(ocs2_bridge library/core/ocs2_bridge/ocs2_bridge.cpp)
    target_include_directories(ocs2_bridge PUBLIC
        library/core/ocs2_bridge
        library/core/logger
        ${ZMQ_INCLUDE_DIRS}
    )
    target_link_libraries(ocs2_bridge PUBLIC ${ZMQ_LIBRARIES} Threads::Threads)
    target_link_directories(ocs2_bridge PUBLIC ${ZMQ_LIBRARY_DIRS})
    # Compiled into every consumer so fsm_go2_x5.hpp can gate the OCS2 state.
    target_compile_definitions(ocs2_bridge PUBLIC USE_OCS2_BRIDGE)

    if(NOT USE_CMAKE)
        if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
            install(TARGETS ocs2_bridge DESTINATION lib/${PROJECT_NAME})
        endif()
    endif()
endif()

# library for inference runtime
add_library(inference_runtime library/core/inference_runtime/inference_runtime.cpp)
if(USE_TORCH)
    target_link_libraries(inference_runtime PUBLIC ${TORCH_LIBRARIES})
    target_compile_definitions(inference_runtime PRIVATE USE_TORCH)
    # Link OpenMP on macOS (required by LibTorch)
    if(SYSTEM_TYPE STREQUAL "Darwin" AND OPENMP_LIBRARIES)
        target_link_libraries(inference_runtime PUBLIC ${OPENMP_LIBRARIES})
    endif()
endif()
if(USE_ONNX)
    target_link_libraries(inference_runtime PUBLIC ${ONNX_RUNTIME_LIB})
    target_compile_definitions(inference_runtime PRIVATE USE_ONNX)
endif()
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS inference_runtime DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# library for motion loader
add_library(motion_loader library/core/motion_loader/motion_loader.cpp)

# library for rl sdk
add_library(rl_sdk library/core/rl_sdk/rl_sdk.cpp)
target_link_libraries(rl_sdk PUBLIC
    motion_loader
    inference_runtime
    Python3::Python
    Python3::Module
    TBB::tbb
)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
    target_link_libraries(rl_sdk PUBLIC Python3::NumPy)
else()
    target_compile_definitions(rl_sdk PUBLIC WITHOUT_NUMPY)
endif()
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS rl_sdk DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# library for observation buffer
add_library(observation_buffer library/core/observation_buffer/observation_buffer.cpp)
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS observation_buffer DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# executable for rl sim
if(NOT USE_CMAKE)
    add_executable(rl_sim src/rl_sim.cpp)
    target_link_libraries(rl_sim
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
        Threads::Threads
    )
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        target_link_libraries(rl_sim ${catkin_LIBRARIES})
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        ament_target_dependencies(rl_sim
            joint_state_broadcaster
            robot_state_publisher
            rclcpp
            gazebo_ros
            std_msgs
            robot_msgs
            robot_joint_controller
            rclpy
            gazebo_msgs
            std_srvs
            geometry_msgs
        )
        install(TARGETS rl_sim DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# executable for rl real a1
if(SYSTEM_TYPE STREQUAL "Linux")
    add_executable(rl_real_a1 src/rl_real_a1.cpp)
    target_link_libraries(rl_real_a1
        ${UNITREE_A1_LIBS}
        rl_sdk
        observation_buffer
            ${YAML_CPP_LIBRARIES}
    )
    if(NOT USE_CMAKE)
        if($ENV{ROS_DISTRO} MATCHES "noetic")
            target_link_libraries(rl_real_a1 ${catkin_LIBRARIES})
        elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
            ament_target_dependencies(rl_real_a1
                rclcpp
                geometry_msgs
            )
            install(TARGETS rl_real_a1 DESTINATION lib/${PROJECT_NAME})
        endif()
    endif()
endif()

if(SYSTEM_TYPE STREQUAL "Linux")
    add_executable(rl_real_lite3
        src/rl_real_lite3.cpp
        ${GAMEPAD_SRC}
    )
    target_link_libraries(rl_real_lite3
        ${LITE3_REAL_LIBS}
        rl_sdk
        observation_buffer
            ${YAML_CPP_LIBRARIES}
    )
    target_include_directories(rl_real_lite3 PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/deeprobotics/gamepad/include
    )
    if(NOT USE_CMAKE)
        if($ENV{ROS_DISTRO} MATCHES "noetic")
            target_link_libraries(rl_real_lite3 ${catkin_LIBRARIES})
        elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
            ament_target_dependencies(rl_real_lite3
                rclcpp
                geometry_msgs
            )
            install(TARGETS rl_real_lite3 DESTINATION lib/${PROJECT_NAME})
        endif()
    endif()
endif()

# executable for rl real go2
if(SYSTEM_TYPE STREQUAL "Linux")
    add_executable(rl_real_go2 src/rl_real_go2.cpp)
    target_link_libraries(rl_real_go2
        unitree_sdk2
        rl_sdk
        observation_buffer
            ${YAML_CPP_LIBRARIES}
    )
    # Add unitree_sdk2 library path to RPATH
    set_target_properties(rl_real_go2 PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/unitree/unitree_sdk2/thirdparty/lib/${ARCH_DIR}"
    )
    if(NOT USE_CMAKE AND "$ENV{ROS_DISTRO}" MATCHES "foxy|humble")
        install(TARGETS rl_real_go2 DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# Pure ROS 2 transport for Go2/Go2W/Go2-X5. This target deliberately does not
# link unitree_sdk2 or its bundled CycloneDDS; Unitree topics are accessed via
# the unitree_go and unitree_api ROS interfaces in domain 0.
if(SYSTEM_TYPE STREQUAL "Linux" AND NOT USE_CMAKE AND "$ENV{ROS_DISTRO}" MATCHES "foxy|humble")
    add_executable(rl_real_go2_ros2 src/rl_real_go2_ros2.cpp)
    target_link_libraries(rl_real_go2_ros2
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
    )
    ament_target_dependencies(rl_real_go2_ros2
        rclcpp
        geometry_msgs
        nav_msgs
        unitree_api
        unitree_go
    )
    if($ENV{ROS_DISTRO} STREQUAL "foxy")
        target_compile_definitions(rl_real_go2_ros2 PRIVATE ROS_DISTRO_FOXY)
    endif()
    install(TARGETS rl_real_go2_ros2 DESTINATION lib/${PROJECT_NAME})
endif()

# executable for rl real g1
if(SYSTEM_TYPE STREQUAL "Linux")
    add_executable(rl_real_g1 src/rl_real_g1.cpp)
    target_include_directories(rl_real_g1 BEFORE PRIVATE ${UNITREE_SDK2_INCLUDE_DIRS})
    target_link_libraries(rl_real_g1
        unitree_sdk2
        rl_sdk
        observation_buffer
            ${YAML_CPP_LIBRARIES}
    )
    # Add unitree_sdk2 library path to RPATH
    set_target_properties(rl_real_g1 PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/unitree/unitree_sdk2/thirdparty/lib/${ARCH_DIR}"
    )
    if(NOT USE_CMAKE)
        if($ENV{ROS_DISTRO} MATCHES "noetic")
            target_link_libraries(rl_real_g1 ${catkin_LIBRARIES})
        elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
            ament_target_dependencies(rl_real_g1
                rclcpp
                geometry_msgs
            )
            install(TARGETS rl_real_g1 DESTINATION lib/${PROJECT_NAME})
        endif()
    endif()
endif()

# executable for rl real l4w4
if(SYSTEM_TYPE STREQUAL "Linux")
    add_executable(rl_real_l4w4 src/rl_real_l4w4.cpp)
    target_link_libraries(rl_real_l4w4
        l4w4_sdk
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
    )
    if(NOT USE_CMAKE)
        if($ENV{ROS_DISTRO} MATCHES "noetic")
            target_link_libraries(rl_real_l4w4 ${catkin_LIBRARIES})
        elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
            ament_target_dependencies(rl_real_l4w4
                rclcpp
                geometry_msgs
            )
            install(TARGETS rl_real_l4w4 DESTINATION lib/${PROJECT_NAME})
        endif()
    endif()
endif()

# executable for rl real d1
if(SYSTEM_TYPE STREQUAL "Linux" AND BUILD_AGIBOT_D1)
    add_executable(rl_real_d1 src/rl_real_d1.cpp)
    target_link_libraries(rl_real_d1
        ${AGIBOT_D1_LIBS}
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
    )
    # Add D1 SDK library path to RPATH
    set_target_properties(rl_real_d1 PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/agibot/agibot_D1_Edu-Ultra/lib/zsl-1/${ARCH_DIR}"
    )
    if(NOT USE_CMAKE)
        if($ENV{ROS_DISTRO} MATCHES "noetic")
            target_link_libraries(rl_real_d1 ${catkin_LIBRARIES})
        elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
            ament_target_dependencies(rl_real_d1
                rclcpp
                geometry_msgs
            )
            install(TARGETS rl_real_d1 DESTINATION lib/${PROJECT_NAME})
        endif()
    endif()
endif()

# executable for actuator net
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        catkin_install_python(PROGRAMS
            scripts/actuator_net.py
            DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(PROGRAMS
            scripts/actuator_net.py
            DESTINATION lib/${PROJECT_NAME}
        )
    endif()
endif()

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(
            DIRECTORY launch worlds ${PROJECT_ROOT_DIR}/policy
            DESTINATION share/${PROJECT_NAME}/${dir}
        )
    endif()
endif()

# ========================
# mujoco
# ========================
if(USE_MUJOCO)
    add_compile_definitions(USE_MUJOCO)
    set(MUJOCO_DIR "${PROJECT_ROOT_DIR}/library/mujoco")

    # Check MuJoCo installation (different paths for different platforms)
    if(SYSTEM_TYPE STREQUAL "Darwin")
        # macOS uses framework structure
        if(NOT EXISTS "${MUJOCO_DIR}/Headers/mujoco.h")
            message(FATAL_ERROR "MuJoCo not found. Please run: bash scripts/download_mujoco.sh")
        endif()

        # Create compatible include structure in build directory for macOS
        # This allows <mujoco/mujoco.h> to work on macOS where headers are in Headers/ directory
        set(MUJOCO_COMPAT_INCLUDE_DIR "${CMAKE_BINARY_DIR}/mujoco_include")
        file(MAKE_DIRECTORY "${MUJOCO_COMPAT_INCLUDE_DIR}/mujoco")
        file(GLOB MUJOCO_HEADERS "${MUJOCO_DIR}/Headers/*.h")
        file(COPY ${MUJOCO_HEADERS} DESTINATION "${MUJOCO_COMPAT_INCLUDE_DIR}/mujoco")

        set(MUJOCO_INCLUDE_DIR "${MUJOCO_COMPAT_INCLUDE_DIR}")
        set(MUJOCO_LIB_DIR "${MUJOCO_DIR}/Versions/Current")
    else()
        # Linux/Windows use standard structure
        if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h")
            message(FATAL_ERROR "MuJoCo not found. Please run: bash scripts/download_mujoco.sh")
        endif()
        set(MUJOCO_INCLUDE_DIR "${MUJOCO_DIR}/include")
        set(MUJOCO_LIB_DIR "${MUJOCO_DIR}/lib")
    endif()

    # Add MuJoCo library path to global RPATH
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${MUJOCO_LIB_DIR}")
    message(STATUS "Added MuJoCo to RPATH: ${MUJOCO_LIB_DIR}")

    # Find GLFW
    find_package(glfw3 REQUIRED)

    # Platform-specific library names
    if(SYSTEM_TYPE STREQUAL "Linux")
        set(MUJOCO_LIB_NAME "libmujoco.so.3.2.7")
    elseif(SYSTEM_TYPE STREQUAL "Darwin")
        set(MUJOCO_LIB_NAME "libmujoco.3.2.7.dylib")
    elseif(SYSTEM_TYPE STREQUAL "Windows")
        set(MUJOCO_LIB_NAME "mujoco.dll")
    else()
        message(FATAL_ERROR "Unsupported platform: ${SYSTEM_TYPE}")
    endif()

    # Import MuJoCo library with RPATH
    add_library(mujoco_prebuilt SHARED IMPORTED)
    set_target_properties(mujoco_prebuilt PROPERTIES
        IMPORTED_LOCATION "${MUJOCO_LIB_DIR}/${MUJOCO_LIB_NAME}"
        INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
        IMPORTED_NO_SONAME TRUE
    )

    # Include directories
    include_directories(
        ${MUJOCO_INCLUDE_DIR}
        library/thirdparty/joystick
    )

    # Source files
    file(GLOB MUJOCO_SIMULATE_SRC library/thirdparty/mujoco_simulate/*.cc)
    set(JOYSTICK_SRC library/thirdparty/joystick/joystick.cc)

    # On macOS, also include Objective-C++ files for CoreVideo support
    if(SYSTEM_TYPE STREQUAL "Darwin")
        file(GLOB MUJOCO_SIMULATE_MM library/thirdparty/mujoco_simulate/*.mm)
        list(APPEND MUJOCO_SIMULATE_SRC ${MUJOCO_SIMULATE_MM})
    endif()

    # Build executable
    add_executable(rl_sim_mujoco src/rl_sim_mujoco.cpp ${MUJOCO_SIMULATE_SRC} ${JOYSTICK_SRC})

    # Deterministic, bounded physics evaluation through the same RL SDK.
    # This executable has no viewer, joystick, ROS, or hardware output path.
    add_executable(rl_mujoco_eval test/mujoco_closed_loop.cpp)
    target_link_libraries(rl_mujoco_eval
        rl_sdk observation_buffer ${YAML_CPP_LIBRARIES}
        Threads::Threads mujoco_prebuilt
    )

    add_executable(rl_gait_observation_probe test/gait_observation_probe.cpp)
    target_link_libraries(rl_gait_observation_probe
        rl_sdk observation_buffer ${YAML_CPP_LIBRARIES} Threads::Threads
    )

    # Suppress deprecation warnings for macOS 15.0 API changes in MuJoCo Simulate
    if(SYSTEM_TYPE STREQUAL "Darwin")
        set_source_files_properties(
            ${MUJOCO_SIMULATE_MM}
            PROPERTIES COMPILE_FLAGS "-Wno-deprecated-declarations"
        )
    endif()

    # Add include directories for mujoco_simulate and joystick headers
    target_include_directories(rl_sim_mujoco PRIVATE
        library/thirdparty/mujoco_simulate
        library/thirdparty/joystick
    )

    target_link_libraries(rl_sim_mujoco
        rl_sdk observation_buffer ${YAML_CPP_LIBRARIES}
        Threads::Threads mujoco_prebuilt glfw
    )

    if(USE_OCS2_BRIDGE)
        target_link_libraries(rl_sim_mujoco ocs2_bridge)
    endif()

    if(USE_JOYLINK)
        if(NOT JOYLINK_ROOT)
            message(FATAL_ERROR "USE_JOYLINK=ON requires -DJOYLINK_ROOT=/path/to/JoyLink (a checkout already built with -DBUILD_ZMQ=ON)")
        endif()
        set(JOYLINK_CLIENT_LIB "${JOYLINK_ROOT}/build/client/libjoylink_client.a")
        set(JOYLINK_COMMON_LIB "${JOYLINK_ROOT}/build/common/libjoystick_common.a")
        set(JOYLINK_YAML_CPP_LIB "${JOYLINK_ROOT}/build/_deps/yaml-cpp-build/libyaml-cpp.a")
        foreach(joylink_lib ${JOYLINK_CLIENT_LIB} ${JOYLINK_COMMON_LIB} ${JOYLINK_YAML_CPP_LIB})
            if(NOT EXISTS "${joylink_lib}")
                message(FATAL_ERROR "JoyLink artifact not found: ${joylink_lib} -- build JoyLink first (cmake -S . -B build -DBUILD_ZMQ=ON && cmake --build build)")
            endif()
        endforeach()

        find_package(PkgConfig REQUIRED)
        pkg_check_modules(ZMQ REQUIRED libzmq)

        target_compile_definitions(rl_sim_mujoco PRIVATE USE_JOYLINK)
        target_include_directories(rl_sim_mujoco PRIVATE
            ${JOYLINK_ROOT}/client/include
            ${JOYLINK_ROOT}/include
            ${ZMQ_INCLUDE_DIRS}
        )
        target_link_libraries(rl_sim_mujoco
            ${JOYLINK_CLIENT_LIB} ${JOYLINK_COMMON_LIB} ${JOYLINK_YAML_CPP_LIB}
            ${ZMQ_LIBRARIES}
        )
        target_link_directories(rl_sim_mujoco PRIVATE ${ZMQ_LIBRARY_DIRS})
        message(STATUS "JoyLink gamepad support enabled: ${JOYLINK_ROOT}")
    endif()

    # macOS specific frameworks and libraries
    if(SYSTEM_TYPE STREQUAL "Darwin")
        # OpenMP support
        if(OPENMP_LIBRARIES)
            target_link_libraries(rl_sim_mujoco ${OPENMP_LIBRARIES})
        endif()

        # Required frameworks for GlfwCoreVideo
        target_link_libraries(rl_sim_mujoco
            "-framework CoreVideo"
            "-framework OpenGL"
            "-framework AppKit"
        )
    endif()

    install(TARGETS rl_sim_mujoco DESTINATION bin)
    message(STATUS "MuJoCo configured: ${MUJOCO_DIR}")
endif()

# ========================
# only for test
# ========================
# add_executable(test_observation_buffer test/test_observation_buffer.cpp)
# target_link_libraries(test_observation_buffer
#     observation_buffer
# )
# install(TARGETS test_observation_buffer DESTINATION lib/${PROJECT_NAME})
# add_executable(test_vector_math test/test_vector_math.cpp)
# install(TARGETS test_vector_math DESTINATION lib/${PROJECT_NAME})

# add_executable(test_inference_stability test/test_inference_stability.cpp)
# target_link_libraries(test_inference_stability
#     inference_runtime
#     ${TORCH_LIBRARIES}
#     ${ONNX_RUNTIME_LIB}
# )
# install(TARGETS test_inference_stability DESTINATION lib/${PROJECT_NAME})

message(STATUS "========================================")
message(STATUS "       RL_SAR Build Configuration       ")
message(STATUS "========================================")
message(STATUS "Project: ${PROJECT_NAME} v${PROJECT_VERSION}")
message(STATUS "System: ${SYSTEM_TYPE}")
message(STATUS "Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "USE_CMAKE: ${USE_CMAKE}")
message(STATUS "----------------------------------------")
message(STATUS "Backend Configuration:")
message(STATUS "USE_TORCH: ${USE_TORCH}")
message(STATUS "USE_ONNX: ${USE_ONNX}")
if(USE_TORCH)
    if(Torch_VERSION)
        message(STATUS "LibTorch version: ${Torch_VERSION}")
    else()
        message(STATUS "LibTorch: Available")
    endif()
    if(IS_JETSON)
        message(STATUS "LibTorch: Jetson device detected")
        if(JETPACK_VERSION)
            message(STATUS "JetPack version: ${JETPACK_VERSION}")
        endif()
    endif()
endif()
if(USE_ONNX)
    if(ONNX_RUNTIME_VERSION)
        message(STATUS "ONNX Runtime version: ${ONNX_RUNTIME_VERSION}")
    elseif(onnxruntime_VERSION)
        message(STATUS "ONNX Runtime version: ${onnxruntime_VERSION}")
    else()
        message(STATUS "ONNX Runtime: Available")
    endif()
endif()
message(STATUS "----------------------------------------")
message(STATUS "Simulator Configuration:")
message(STATUS "USE_MUJOCO: ${USE_MUJOCO}")
if(USE_MUJOCO)
    message(STATUS "MuJoCo: Enabled")
endif()
message(STATUS "USE_OCS2_BRIDGE: ${USE_OCS2_BRIDGE}")
if(USE_OCS2_BRIDGE)
    message(STATUS "ZeroMQ version: ${ZMQ_VERSION}")
endif()
message(STATUS "========================================")
