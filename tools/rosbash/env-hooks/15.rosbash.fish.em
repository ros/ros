# generated from rosbash/env-hooks/15.rosbash.fish.em

@[if DEVELSPACE]@
source "@(CMAKE_CURRENT_SOURCE_DIR)/rosfish"
@[else]@
if test -z "$CATKIN_ENV_HOOK_WORKSPACE"
    set CATKIN_ENV_HOOK_WORKSPACE "@(CMAKE_INSTALL_PREFIX)"
end
source "$CATKIN_ENV_HOOK_WORKSPACE/share/rosbash/rosfish"
@[end if]@
