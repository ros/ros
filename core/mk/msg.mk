gen_msgs:
	$(shell rospack find roslib)/scripts/genmsg msg/*.msg

clean_msgs:
	-rm -rf msg/cpp

gen_srv:
	$(shell rospack find roslib)/scripts/gensrv srv/*.srv

clean_srv:
	-rm -rf srv/cpp
