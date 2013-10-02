# generated from rosbash/env-hooks/15.rosbash.zsh.em

@[if DEVELSPACE]@
. "@(CMAKE_CURRENT_SOURCE_DIR)/roszsh"
@[else]@
if [ -z "$CATKIN_ENV_HOOK_WORKSPACE" ]; then
  CATKIN_ENV_HOOK_WORKSPACE="@(CMAKE_INSTALL_PREFIX)"
fi
. "$CATKIN_ENV_HOOK_WORKSPACE/share/rosbash/roszsh"
@[end if]@
