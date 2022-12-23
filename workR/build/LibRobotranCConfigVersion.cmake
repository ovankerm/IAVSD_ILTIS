
# - Version Config file for the Robotran MBSysC package
# It defines the version of the Robotran MBSysC package and
# checks the compatibility with the user projects

##############################
# Set version
##############################


set(PACKAGE_VERSION 1.20.0)

##############################
# Set compatibility rule
##############################

# compatibility for the "find_package( LibRobotranC X.Y.Z EXACT )"
# set to the exact same version number

if (PACKAGE_FIND_VERSION VERSION_EQUAL PACKAGE_VERSION)
    set(PACKAGE_VERSION_EXACT TRUE)
endif (PACKAGE_FIND_VERSION VERSION_EQUAL PACKAGE_VERSION)


# compatibility for the "find_package( LibRobotranC X.Y.Z )"
# set to any version X.Y.*

if("${PACKAGE_FIND_VERSION_MAJOR}" EQUAL 1)
  if("${PACKAGE_FIND_VERSION_MINOR}" EQUAL 20)
    set(PACKAGE_VERSION_COMPATIBLE TRUE)
    endif()
endif()
