cmake_minimum_required(VERSION 3.14)
project(DeviationCorrector VERSION 1.0.0 LANGUAGES CXX)

# C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译选项
if(MSVC)
    add_compile_options(/W4 /utf-8)
    # 设置运行时库
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
else()
    add_compile_options(-Wall -Wextra -Wpedantic -fvisibility=hidden)
endif()

# ==================== 依赖配置 ====================

# 优先使用本地 thirdparty 中的 Eigen3 (git submodule)
set(EIGEN3_LOCAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/eigen")

if(EXISTS "${EIGEN3_LOCAL_PATH}/Eigen")
    # 使用本地 Eigen3 (header-only)
    message(STATUS "Using local Eigen3 from: ${EIGEN3_LOCAL_PATH}")
    set(EIGEN3_INCLUDE_DIR "${EIGEN3_LOCAL_PATH}")
    set(EIGEN3_FOUND TRUE)
    
    # 创建 Eigen3::Eigen 接口库 (兼容 find_package 方式)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    target_include_directories(Eigen3::Eigen INTERFACE "${EIGEN3_LOCAL_PATH}")
else()
    # 回退到系统 Eigen3
    message(STATUS "Using system Eigen3")
    find_package(Eigen3 3.3 REQUIRED)
endif()

# ==================== 动态库目标 ====================

add_library(deviation_corrector SHARED
    src/deviation_corrector.cpp
)

# 设置输出名称
set_target_properties(deviation_corrector PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    OUTPUT_NAME "deviation_corrector"
    WINDOWS_EXPORT_ALL_SYMBOLS OFF
)

# 定义导出宏
target_compile_definitions(deviation_corrector
    PRIVATE
        DEVIATION_CORRECTOR_EXPORTS
)

target_include_directories(deviation_corrector
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(deviation_corrector
    PUBLIC
        Eigen3::Eigen
)

# Windows 平台特殊处理
if(WIN32)
    # 确保 DLL 和 LIB 输出到同一目录
    set_target_properties(deviation_corrector PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/$<CONFIG>
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/$<CONFIG>
        ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/$<CONFIG>
    )
endif()

# ==================== 测试目标 ====================

option(BUILD_TESTS "Build unit tests" ON)

if(BUILD_TESTS)
    add_executable(test_deviation_corrector
        test/test_deviation_corrector.cpp
    )
    
    target_link_libraries(test_deviation_corrector
        PRIVATE
            deviation_corrector
    )
    
    # 设置测试输出目录
    set_target_properties(test_deviation_corrector PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/$<CONFIG>
    )
    
    # 添加测试
    enable_testing()
    add_test(NAME DeviationCorrectorTests COMMAND test_deviation_corrector)
endif()

# ==================== 示例程序 ====================

option(BUILD_EXAMPLES "Build example programs" ON)

if(BUILD_EXAMPLES)
    # 单点纠偏示例
    add_executable(example_single_correction
        examples/example_single_correction.cpp
    )
    target_link_libraries(example_single_correction PRIVATE deviation_corrector)
    set_target_properties(example_single_correction PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/$<CONFIG>
    )
    
    # 多点位示例
    add_executable(example_multi_point
        examples/example_multi_point.cpp
    )
    target_link_libraries(example_multi_point PRIVATE deviation_corrector)
    set_target_properties(example_multi_point PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/$<CONFIG>
    )
endif()

# ==================== 安装配置 ====================

include(GNUInstallDirs)

install(TARGETS deviation_corrector
    EXPORT deviation_corrector-targets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT deviation_corrector-targets
    FILE deviation_corrector-targets.cmake
    NAMESPACE deviation_corrector::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/deviation_corrector
)

# 版本配置
include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/deviation_corrector-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/deviation_corrector-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/deviation_corrector
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/deviation_corrector-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/deviation_corrector-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/deviation_corrector-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/deviation_corrector
)

# ==================== 打印配置信息 ====================

message(STATUS "")
message(STATUS "DeviationCorrector Configuration:")
message(STATUS "  Version:       ${PROJECT_VERSION}")
message(STATUS "  C++ Standard:  ${CMAKE_CXX_STANDARD}")
message(STATUS "  Library Type:  SHARED (DLL)")
message(STATUS "  Build Tests:   ${BUILD_TESTS}")
message(STATUS "  Build Examples: ${BUILD_EXAMPLES}")
message(STATUS "  Eigen3:        ${EIGEN3_INCLUDE_DIR}")
message(STATUS "  Output Dir:    ${CMAKE_BINARY_DIR}/$<CONFIG>")
message(STATUS "")
