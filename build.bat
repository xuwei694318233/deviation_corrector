@echo off
REM ============================================================
REM DeviationCorrector 构建脚本
REM 用法: build.bat [release|debug|clean|all]
REM ============================================================

setlocal EnableDelayedExpansion

set PROJECT_DIR=%~dp0
REM Remove trailing backslash to avoid quote escaping issues
if "%PROJECT_DIR:~-1%"=="\" set PROJECT_DIR=%PROJECT_DIR:~0,-1%
set BUILD_DIR=%PROJECT_DIR%\build

REM 解析参数
set BUILD_TYPE=%1
if "%BUILD_TYPE%"=="" set BUILD_TYPE=release

echo.
echo ============================================================
echo   DeviationCorrector Build Script
echo   Build Type: %BUILD_TYPE%
echo   Project Dir: %PROJECT_DIR%
echo ============================================================
echo.

REM 检查 CMake 是否可用
where cmake >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo [错误] 未找到 CMake，请确保 CMake 已安装并添加到 PATH
    echo 下载地址: https://cmake.org/download/
    exit /b 1
)

REM 处理不同的构建命令
if /i "%BUILD_TYPE%"=="clean" goto :clean
if /i "%BUILD_TYPE%"=="all" goto :build_all
if /i "%BUILD_TYPE%"=="release" goto :build_release
if /i "%BUILD_TYPE%"=="debug" goto :build_debug
if /i "%BUILD_TYPE%"=="configure" goto :configure

echo [错误] 未知的构建类型: %BUILD_TYPE%
echo 用法: build.bat [release|debug|clean|all|configure]
exit /b 1

:configure
echo [步骤] 配置项目...
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"
cd /d "%BUILD_DIR%"
cmake -G "Visual Studio 17 2022" -A x64 "%PROJECT_DIR%"
if %ERRORLEVEL% neq 0 (
    echo [错误] 配置失败
    exit /b 1
)
echo [完成] 配置成功
goto :end

:build_release
echo [步骤] 构建 Release 版本...
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"
cd /d "%BUILD_DIR%"
cmake -G "Visual Studio 17 2022" -A x64 "%PROJECT_DIR%"
if %ERRORLEVEL% neq 0 (
    echo [错误] 配置失败
    exit /b 1
)
cmake --build . --config Release
if %ERRORLEVEL% neq 0 (
    echo [错误] Release 构建失败
    exit /b 1
)
echo.
echo [完成] Release 构建成功!
echo 输出目录: %BUILD_DIR%\Release
goto :run_tests

:build_debug
echo [步骤] 构建 Debug 版本...
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"
cd /d "%BUILD_DIR%"
cmake -G "Visual Studio 17 2022" -A x64 "%PROJECT_DIR%"
if %ERRORLEVEL% neq 0 (
    echo [错误] 配置失败
    exit /b 1
)
cmake --build . --config Debug
if %ERRORLEVEL% neq 0 (
    echo [错误] Debug 构建失败
    exit /b 1
)
echo.
echo [完成] Debug 构建成功!
echo 输出目录: %BUILD_DIR%\Debug
goto :run_tests

:build_all
echo [步骤] 构建 Release 和 Debug 版本...
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"
cd /d "%BUILD_DIR%"
cmake -G "Visual Studio 17 2022" -A x64 "%PROJECT_DIR%"
if %ERRORLEVEL% neq 0 (
    echo [错误] 配置失败
    exit /b 1
)
echo.
echo [构建] Release 版本...
cmake --build . --config Release
if %ERRORLEVEL% neq 0 (
    echo [错误] Release 构建失败
    exit /b 1
)
echo.
echo [构建] Debug 版本...
cmake --build . --config Debug
if %ERRORLEVEL% neq 0 (
    echo [错误] Debug 构建失败
    exit /b 1
)
echo.
echo [完成] 所有版本构建成功!
goto :end

:clean
echo [步骤] 清理构建目录...
if exist "%BUILD_DIR%" (
    rmdir /s /q "%BUILD_DIR%"
    echo [完成] 构建目录已清理
) else (
    echo [提示] 构建目录不存在，无需清理
)
goto :end

:run_tests
echo.
echo [步骤] 运行测试...
if exist "%BUILD_DIR%\%BUILD_TYPE%\test_deviation_corrector.exe" (
    "%BUILD_DIR%\%BUILD_TYPE%\test_deviation_corrector.exe"
) else if exist "%BUILD_DIR%\Debug\test_deviation_corrector.exe" (
    "%BUILD_DIR%\Debug\test_deviation_corrector.exe"
) else if exist "%BUILD_DIR%\Release\test_deviation_corrector.exe" (
    "%BUILD_DIR%\Release\test_deviation_corrector.exe"
) else (
    echo [提示] 测试程序未找到
)
goto :end

:end
echo.
echo ============================================================
echo   构建完成
echo ============================================================
endlocal
