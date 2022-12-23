
# - Config file for the Robotran MBSysC package
# It defines the following variables
#  Robotran libraries to link against
#      * LIB_MBSYSC_MODULES
#    * LIB_MBSYSC_LOAD
#    * LIB_MBSYSC_UTILITIES
#    * LIB_MBSYSC_REALTIME
#  LIB_MBSYSC_INCLUDE_DIRS - Directories containing the headers necessary to use MBSysC libraries
#  LIB_MBSYSC_DEFINITIONS  - Definitions used to compile MBSysC libraries, shall be used as well by the project linking to the MBSysC libraries (so that headers matches)

#############
# LIBRARIES
#############

# MBSYSC_MODULES
FIND_LIBRARY(LIB_MBSYSC_MODULES MBsysC_module
    PATHS 
          //Release
          //Debug
          /Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/mbs_module
          /Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/mbs_module//Debug
          /Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/mbs_module//Release
)

# MBSYSC_LOAD
FIND_LIBRARY(LIB_MBSYSC_LOAD MBsysC_loadXML
    PATHS 
          //Release
          //Debug
          /Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/mbs_load_xml
          /Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/mbs_load_xml//Debug
          /Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/mbs_load_xml//Release
)

# MBSYSC_UTILITIES
FIND_LIBRARY(LIB_MBSYSC_UTILITIES MBsysC_utilities
    PATHS 
          //Release
          //Debug
          /Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/mbs_utilities
          /Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/mbs_utilities//Debug
          /Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/mbs_utilities//Release
)

# MBSYSC_REALTIME
FIND_LIBRARY(LIB_MBSYSC_REALTIME MBsysC_realtime
    PATHS 
          //Release
          //Debug
          /Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/mbs_realtime
          /Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/mbs_realtime//Debug
          /Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/workR/build/mbs_common/mbs_realtime//Release
)


#############
# USEFUL
#############

# Path to Robotran common files
SET(ROBOTRAN_SOURCE_DIR /Users/vankermotis/.robotran/mbsysc/MBsysC/mbs_common/../)

# Call project funstion (symbolic and user) via function pointers
SET(FLAG_PRJ_FCT_PTR OFF)

# Realtime options
SET(FLAG_REAL_TIME OFF)
SET(FLAG_PLOT OFF)
SET(FLAG_VISU OFF)
SET(FLAG_JAVA OFF)
SET(FLAG_OPEN_GL OFF)

# Shared lib compilation
SET(FLAG_SHARED_LIB OFF)

#############
# INCLUDE DIRECTORIES
#############

SET(LIB_MBSYSC_INCLUDE_DIRS /Users/vankermotis/.robotran/mbsysc/MBsysC/mbs_common/..//mbs_common/mbs_struct;/Users/vankermotis/.robotran/mbsysc/MBsysC/mbs_common/..//mbs_common/mbs_load_xml;/Users/vankermotis/.robotran/mbsysc/MBsysC/mbs_common/..//mbs_common/mbs_module;/Users/vankermotis/.robotran/mbsysc/MBsysC/mbs_common/..//mbs_common/mbs_utilities;/Users/vankermotis/.robotran/mbsysc/MBsysC/mbs_common/..//mbs_common/mbs_utilities/auto_output;/Users/vankermotis/.robotran/mbsysc/MBsysC/mbs_common/..//mbs_common/mbs_realtime;/Users/vankermotis/.robotran/mbsysc/MBsysC/mbs_common/..//mbs_common/mbs_realtime/realtime;/Users/vankermotis/.robotran/mbsysc/MBsysC/mbs_common/..//mbs_common/mbs_realtime/sdl;/Users/vankermotis/.robotran/mbsysc/MBsysC/mbs_common/..//mbs_common/mbs_realtime/sdl/auto_plot;/Users/vankermotis/.robotran/mbsysc/MBsysC/mbs_common/..//mbs_common/mbs_numerics )

#############
# DEFINITIONS
#############

SET(LIB_MBSYSC_DEFINITIONS   -DUNIX)

