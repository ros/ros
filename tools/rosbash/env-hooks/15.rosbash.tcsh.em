# generated from rosbash/env-hooks/15.rosbash.tcsh.em

@[if BUILDSPACE]@
. @(CMAKE_CURRENT_SOURCE_DIR)/rostcsh
@[else]@
. @(CMAKE_INSTALL_PREFIX)/share/rosbash/rostcsh
@[end if]@
