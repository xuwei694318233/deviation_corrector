#!/usr/bin/env pwsh
# ============================================================
# DeviationCorrector 构建脚本 (PowerShell 版本)
# 用法: ./build.ps1 [-BuildType release|debug|clean|all|configure]
# ============================================================

param(
    [Parameter(Position=0)]
    [ValidateSet("release", "debug", "clean", "all", "configure", "test")]
    [string]$BuildType = "release"
)

$ProjectDir = $PSScriptRoot
$BuildDir = Join-Path $ProjectDir "build"
$CMakeGenerator = "Visual Studio 17 2022"
$Arch = "x64"

Write-Host ""
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host "  DeviationCorrector Build Script (PowerShell)" -ForegroundColor Cyan
Write-Host "  Build Type: $BuildType" -ForegroundColor Cyan
Write-Host "  Project Dir: $ProjectDir" -ForegroundColor Cyan
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host ""

# 检查 CMake
function Test-CMake {
    $cmake = Get-Command cmake -ErrorAction SilentlyContinue
    if (-not $cmake) {
        Write-Host "[错误] 未找到 CMake，请确保 CMake 已安装并添加到 PATH" -ForegroundColor Red
        Write-Host "下载地址: https://cmake.org/download/" -ForegroundColor Yellow
        exit 1
    }
    return $true
}

# 配置项目
function Invoke-Configure {
    Write-Host "[步骤] 配置项目..." -ForegroundColor Yellow
    
    if (-not (Test-Path $BuildDir)) {
        New-Item -ItemType Directory -Path $BuildDir -Force | Out-Null
    }
    
    Push-Location $BuildDir
    & cmake $ProjectDir -G $CMakeGenerator -A $Arch
    $result = $LASTEXITCODE
    Pop-Location
    
    if ($result -ne 0) {
        Write-Host "[错误] 配置失败" -ForegroundColor Red
        exit 1
    }
    
    Write-Host "[完成] 配置成功" -ForegroundColor Green
}

# 构建指定版本
function Invoke-Build {
    param([string]$Config)
    
    Write-Host "[步骤] 构建 $Config 版本..." -ForegroundColor Yellow
    
    if (-not (Test-Path $BuildDir)) {
        New-Item -ItemType Directory -Path $BuildDir -Force | Out-Null
    }
    
    Push-Location $BuildDir
    & cmake $ProjectDir -G $CMakeGenerator -A $Arch
    & cmake --build . --config $Config
    $result = $LASTEXITCODE
    Pop-Location
    
    if ($result -ne 0) {
        Write-Host "[错误] $Config 构建失败" -ForegroundColor Red
        exit 1
    }
    
    Write-Host "[完成] $Config 构建成功!" -ForegroundColor Green
    Write-Host "输出目录: $BuildDir\$Config" -ForegroundColor Cyan
}

# 清理
function Invoke-Clean {
    Write-Host "[步骤] 清理构建目录..." -ForegroundColor Yellow
    
    if (Test-Path $BuildDir) {
        Remove-Item -Path $BuildDir -Recurse -Force
        Write-Host "[完成] 构建目录已清理" -ForegroundColor Green
    } else {
        Write-Host "[提示] 构建目录不存在，无需清理" -ForegroundColor Yellow
    }
}

# 运行测试
function Invoke-Tests {
    param([string]$Config)
    
    Write-Host ""
    Write-Host "[步骤] 运行测试..." -ForegroundColor Yellow
    
    $testExe = Join-Path $BuildDir "$Config\test_deviation_corrector.exe"
    
    if (-not (Test-Path $testExe)) {
        # 尝试查找任意配置的测试程序
        $testExe = Get-ChildItem -Path $BuildDir -Filter "test_deviation_corrector.exe" -Recurse | Select-Object -First 1
    }
    
    if ($testExe) {
        & $testExe.FullName
    } else {
        Write-Host "[提示] 测试程序未找到" -ForegroundColor Yellow
    }
}

# 显示帮助
function Show-Help {
    Write-Host ""
    Write-Host "用法: ./build.ps1 [-BuildType <type>]" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "构建类型:" -ForegroundColor Cyan
    Write-Host "  release   - 构建 Release 版本 (默认)"
    Write-Host "  debug     - 构建 Debug 版本"
    Write-Host "  all       - 构建 Release 和 Debug 版本"
    Write-Host "  clean     - 清理构建目录"
    Write-Host "  configure - 仅配置项目"
    Write-Host "  test      - 仅运行测试"
    Write-Host ""
}

# 主逻辑
Test-CMake

switch ($BuildType) {
    "clean" {
        Invoke-Clean
    }
    "configure" {
        Invoke-Configure
    }
    "release" {
        Invoke-Build -Config "Release"
        Invoke-Tests -Config "Release"
    }
    "debug" {
        Invoke-Build -Config "Debug"
        Invoke-Tests -Config "Debug"
    }
    "all" {
        Invoke-Build -Config "Release"
        Write-Host ""
        Invoke-Build -Config "Debug"
    }
    "test" {
        Invoke-Tests -Config "Release"
    }
}

Write-Host ""
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host "  构建完成" -ForegroundColor Green
Write-Host "============================================================" -ForegroundColor Cyan
