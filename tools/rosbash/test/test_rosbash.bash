#! /usr/bin/env bash

# This script is supposed to be a unit test for rosbash, also used for manual checks against other operating systems.

# TODO extend this script

. ../rosbash

echo Testing BASH

. test_zshbash.sh

# roslaunch completion

export COMP_CWORD=1
_roscomplete_launch roslaunch
if [[ ! ${COMPREPLY[@]} =~ rosbash ]]; then
   echo "rosbash package missing from" ${COMPREPLY[@]} ; exit 1
fi
echo success roslaunch pkg

export COMP_WORDS=("roslaunch" "--")
export COMP_CWORD=1
_roscomplete_launch roslaunch "--" roslaunch
if [[ ! ${COMPREPLY[@]} == "--files --args --nodes --find-node --child --local --screen --server_uri --run_id --wait --port --core --pid --dump-params" ]]; then
   echo "roslaunch --options missing" from ${COMPREPLY[@]} ; exit 1
fi
echo success roslaunch --option

export COMP_WORDS=("roslaunch" "roslaunch")
export COMP_CWORD=2
_roscomplete_launch roslaunch roslaunch
if [[ ! ${COMPREPLY[@]} =~ "example.launch" ]]; then
   echo "example.launch missing from " ${COMPREPLY[@]} ; exit 1
fi
echo success roslaunch launchfiles

# if [[ ! ${COMPREPLY[@]} =~ "example.launch" ]]; then
#    echo "example.launch missing from " ${COMPREPLY[@]} ; exit 1
# fi
# echo success 4
