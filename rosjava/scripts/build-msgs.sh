#!/bin/bash

PACKAGES="roslib
	    std_msgs
            geometry_msgs
            sensor_msgs
	    robot_msgs
            robot_srvs
	    robot_actions
            deprecated_msgs
            topological_map
            nav_robot_actions
            visualization_msgs
	    door_msgs
	    mechanism_msgs
            pr2_msgs
            plugs_msgs
            motion_planning_msgs
	    motion_planning_srvs
            pr2_robot_actions
            manipulation_msgs
	    manipulation_srvs
	    pr2_mechanism_controllers
            tf
	    tf_node
	    mapping_msgs
 	    navfn
 	    fk_node
	    "

CP=`rospack find rosjava`/src

cd $CP
find . -name "*.java" | xargs javac -cp $CP
cd ..
make

if [ -n "$1" ]
  then
  if [ "clean" = "$1" ] 
  then
    echo "Making clean"
  else 
    echo "Unknown command-line argument"
    exit 0
  fi
fi
  

for pkg in $PACKAGES
do
  DIR=`rospack find $pkg`
  if [ ! -e "$DIR" ]
  then 
     echo "$pkg not found"
     continue
  fi
  echo $DIR
  cd $DIR
  if [ -n "$1" ]
  then
#    make clean
    rm -r msg/java
    rm -r srv/java
  fi
#  make

  for sub in "msg" "srv"
  do
    DIR2=$DIR/$sub/java
    if [ ! -e "$DIR2" ]
    then
       continue
    fi
    CP=$CP:$DIR2
    cd $DIR2
    find . -name "*.java" | xargs javac -cp $CP
  done
done


# rosmake move_arm move_base tf_node pr2_gazebo fake_localization map_server 2dnav_gazebo teleop_base 2dnav_pr2 3dnav_pr2 tabletop_scripts mechanism_control ompl_planning map_server fk_node joint_waypoint_controller
