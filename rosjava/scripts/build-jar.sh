#!/bin/bash

PACKAGES=" 
           roslib
           std_msgs
           robot_msgs
           robot_srvs
           robot_actions
           diagnostic_msgs
           pr2_power_board
           "

CP=
PARAM=
first=0

DIR=`rospack find rosjava`
if [ ! -e "$DIR" ]
then
  echo "$pkg not found"
  continue
fi
echo "Processing: $DIR"

for sub in "src" "msg/java" "srv/java"
do
 DIR2=$DIR/$sub
 if [ ! -e "$DIR2" ]
 then
    continue
 fi
 echo "Adding: $DIR2"
 CP="$DIR2"
 #PARAM="$PARAM -C $DIR2 *"
 #jar cf ros-$pkg.jar -C $DIR2 .
 if [ $first == 0 ]
 then
   first=1
   jar cf ros.jar -C $DIR2 .
 else
   jar uf ros.jar -C $DIR2 .
 fi
done

for pkg in $PACKAGES
do
 DIR=`rospack find $pkg`
 if [ ! -e "$DIR" ]
 then
    echo "$pkg not found"
    continue
 fi
 echo "Processing: $DIR"

 for sub in "msg" "srv"
 do
   DIR2=$DIR/$sub/java
   if [ ! -e "$DIR2" ]
   then
      continue
   fi
   echo "Adding: $DIR2"
   CP="$DIR2"
   #PARAM="$PARAM -C $DIR2 *"
   #jar cf ros-$pkg.jar -C $DIR2 .
   if [ $first == 0 ]
   then
     first=1
     jar cf ros.jar -C $DIR2 .
   else
     jar uf ros.jar -C $DIR2 .
   fi
 done
done

#jar cmf pkgmanifest ros-pkg.jar *.jar
jar i ros.jar

