# Install script for directory: /Users/vankermotis/.robotran/mbsysc/MBsysC/mbs_common

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/Debug")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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
  set(CMAKE_OBJDUMP "/Library/Developer/CommandLineTools/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/cmake_aux/listing/cmake_install.cmake")
  include("/Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/cmake_aux/flags/cmake_install.cmake")
  include("/Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/cmake_aux/libraries/cmake_install.cmake")
  include("/Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/mbs_numerics/cmake_install.cmake")
  include("/Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/mbs_utilities/cmake_install.cmake")
  include("/Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/mbs_load_xml/cmake_install.cmake")
  include("/Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/mbs_realtime/cmake_install.cmake")
  include("/Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/mbs_module/cmake_install.cmake")

endif()

