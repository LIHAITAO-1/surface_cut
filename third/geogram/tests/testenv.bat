:: Template to set environment variables for the
:: non-regression testing framework.

set VORPALINE_SOURCE_DIR=C:\Users\xmyci\Desktop\TetGeo
set VORPALINE_BUILD_DIR=C:\Users\xmyci\Desktop\TetGeo
set VORPALINE_BIN_DIR=C:\Users\xmyci\Desktop\TetGeo\bin
set VORPALINE_LIB_DIR=C:\Users\xmyci\Desktop\TetGeo\lib
set VORPATEST_ROOT_DIR=C:\Users\xmyci\Desktop\TetGeo/tests
set DATADIR=C:\Users\xmyci\Desktop\TetGeo/tests/data
:: set PATH=%PATH%;%VORPALINE_BIN_DIR%

:: Parse arguments
:: If the build directory contains a Makefile, we are in a single build
:: configuration determined at configuration time.
:: Otherwise, we are in a multi-build configuration and the caller must
:: specify which configuration on the command line to use with option --config

if exist %VORPALINE_BUILD_DIR%/Makefile (
    set VORPALINE_BUILD_CONFIG=Debug
) else (
    if "%1" == "--config" (
        call :set_config %2
        shift
        shift
    ) else (
        echo Error: option --config BUILD_CONFIG must be specified as first argument,
        echo where BUILD_CONFIG is the cmake build configuration to use
        exit /B 1
    )
)

:: Swallow the remaining arguments

:options
if "%1" == "" goto :eof
set args=%args% %1
shift
goto options

:: Set Vorpaline executables

:set_config
set VORPALINE_BUILD_CONFIG=%1
set VORPALINE_BIN_DIR=%VORPALINE_BIN_DIR%\%1
set VORPALINE_LIB_DIR=%VORPALINE_LIB_DIR%\%1
goto :eof

