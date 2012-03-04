#! /usr/bin/env zsh

# This script is supposed to be a unit test for rosbash, also used for manual checks against other operating systems.

# TODO extend this script


. ../roszsh

echo "Testing ZSH"

. ./test_zshbash.sh

# roslaunch completion

BUFFER=("foo roslaunch")
_roscomplete_search_dir
if [[ ! ${reply[@]} =~ "example.launch" ]]; then
   echo "rosbash package missing from" ${reply[@]} ; exit 1
fi
echo success roslaunch launchfiles
