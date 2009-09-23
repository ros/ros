#!/bin/bash
rm -rf msg/cpp msg/test_cpp src
`rospack find roslib`/scripts/genmsg msg/*.msg
`rospack find genmsg_cpp`/genmsgtest msg/*.msg
`rospack find roscpp`/scripts/genmsgtest msg/test_cpp
