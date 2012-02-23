#! /usr/bin/env bash

# This script is supposed to be a unit test for rosbash, also used for manual checks against other operating systems.

# TODO extend this script


. ../rosbash

export COMP_CWORD=1
_roscomplete_launch roslaunch
if [[ ! ${COMPREPLY[@]} =~ rosbash ]]; then
   echo "rosbash package missing from" ${COMPREPLY[@]} ; exit 1
fi
echo success 1

export COMP_WORDS=("roslaunch" "--")
export COMP_CWORD=1
_roscomplete_launch roslaunch "--" roslaunch
if [[ ! ${COMPREPLY[@]} == "--files --args --nodes --find-node --child --local --screen --server_uri --run_id --wait --port --core --pid --dump-params" ]]; then
   echo "roslaunch --options missing" from ${COMPREPLY[@]} ; exit 1
fi
echo success 2

export COMP_WORDS=("roslaunch" "roslaunch")
export COMP_CWORD=2
_roscomplete_launch roslaunch roslaunch
if [[ ! ${COMPREPLY[@]} =~ "example.launch" ]]; then
   echo "example.launch missing from " ${COMPREPLY[@]} ; exit 1
fi
echo success 3
