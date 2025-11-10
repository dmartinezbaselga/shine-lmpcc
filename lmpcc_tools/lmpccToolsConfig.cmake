# - Config file for the lmpcc_tools package
# It defines the following variables
#  lmpcc_tools_INCLUDE_DIRS - include directories for lmpcc_tools
#  lmpcc_tools_LIBRARIES    - libraries to link against

# Compute paths
get_filename_component(lmpcc_tools_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(lmpcc_tools_INCLUDE_DIRS "${lmpcc_tools_CMAKE_DIR}/../../../include/lmpcc_tools/include")

set(lmpcc_tools_LIBRARIES "${lmpcc_tools_CMAKE_DIR}/../../../lib/liblmpcc_tools.so")