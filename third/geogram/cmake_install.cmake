# Install script for directory: C:/Users/xmyci/Desktop/TetGeo/third/geogram

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Users/xmyci/Desktop/TetGeo/out/install/x64-Debug")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "CMAKE_OBJDUMP-NOTFOUND")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "devkit" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/modules" TYPE FILE FILES "C:/Users/xmyci/Desktop/TetGeo/third/geogram/cmake/FindGeogram.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("C:/Users/xmyci/Desktop/TetGeo/third/geogram/src/lib/geogram/cmake_install.cmake")
  include("C:/Users/xmyci/Desktop/TetGeo/third/geogram/src/lib/third_party/cmake_install.cmake")
  include("C:/Users/xmyci/Desktop/TetGeo/third/geogram/src/lib/geogram_gfx/cmake_install.cmake")
  include("C:/Users/xmyci/Desktop/TetGeo/third/geogram/src/bin/cmake_install.cmake")
  include("C:/Users/xmyci/Desktop/TetGeo/third/geogram/src/tests/cmake_install.cmake")
  include("C:/Users/xmyci/Desktop/TetGeo/third/geogram/src/examples/cmake_install.cmake")
  include("C:/Users/xmyci/Desktop/TetGeo/third/geogram/tests/cmake_install.cmake")
  include("C:/Users/xmyci/Desktop/TetGeo/third/geogram/doc/cmake_install.cmake")

endif()

