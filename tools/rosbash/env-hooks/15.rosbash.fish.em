# generated from rosbash/env-hooks/15.rosbash.fish.em

@[if DEVELSPACE]@
. "@(CMAKE_CURRENT_SOURCE_DIR)/rosfish"
@[else]@
. "@(CMAKE_INSTALL_PREFIX)/share/rosbash/rosfish"
@[end if]@
