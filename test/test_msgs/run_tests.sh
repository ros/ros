#!/bin/bash
rm -rf msg/cpp msg/test_cpp src
`rospack find roslib`/scripts/genmsg msgtest/*.msg
`rospack find genmsg_cpp`/genmsgtest msgtest/*.msg
`rospack find roscpp`/scripts/genmsgtest msgtest/test_cpp
