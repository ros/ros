# tests that can work against both bash and zsh



# rossed (extended regular expressions)
if [[ ! `echo abcde | _rossed 's,abc?,foo,'` = "foode" ]]; then
   echo "rosbash package missing from" ${reply[@]} ; exit 1
fi

echo success rossed


ROS_LOCATIONS=foohome=/work:share=/share
if [[ ! `_ros_location_find foohome` = "/work" ]]; then
   echo "ros_location_find missed foohome"; exit 1
fi
if [[ ! `_ros_location_find share` = "/share" ]]; then
   echo "ros_location_find missed share"; exit 1
fi

echo success ros_location_find
