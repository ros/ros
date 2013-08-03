# generated from rosbuild/env-hooks/10.rosbuild.sh.em

@[if DEVELSPACE]@
# env variables in develspace
export ROS_ROOT="@(CMAKE_CURRENT_SOURCE_DIR)"
@[else]@
# env variables in installspace
export ROS_ROOT="$CATKIN_ENV_HOOK_WORKSPACE/@(CATKIN_GLOBAL_SHARE_DESTINATION)/ros"
@[end if]@
