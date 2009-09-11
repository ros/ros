from rosbagmigration.rules import *


class update_48627cf39c4b4ad372837fea5f806e57(MessageUpdateRule):
	old_type = "laser_scan/LaserScan"
	old_md5sum = "48627cf39c4b4ad372837fea5f806e57"
	old_full_text = """#
# Laser scans angles are measured counter clockwise, with 0 facing forward
# (along the x-axis) of the device frame
#

Header header
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]
float32 time_increment   # time between measurements [seconds]
float32 scan_time        # time between scans [seconds]
float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]
float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units]

================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id


"""
	new_type = "laser_scan/LaserScan"
	new_md5sum = "b5c92953e2458989cfe48df40f08b188"
	new_full_text = """#
# Laser scans angles are measured counter clockwise, with 0 facing forward
# (along the x-axis) of the device frame
#

Header header
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]
float32 time_increment   # time between measurements [seconds]
float32 scan_time        # time between scans [seconds]
float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]
float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units]
int32 foo
================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

"""

	migrated_types = [
		("Header","Header"),
	]

	valid = False

	def update(self, old_msg, new_msg):
		self.migrate(old_msg.header, new_msg.header)
		new_msg.angle_min = old_msg.angle_min
		new_msg.angle_max = old_msg.angle_max
		new_msg.angle_increment = old_msg.angle_increment
		new_msg.time_increment = old_msg.time_increment
		new_msg.scan_time = old_msg.scan_time
		new_msg.range_min = old_msg.range_min
		new_msg.range_max = old_msg.range_max
		new_msg.ranges = old_msg.ranges
		new_msg.intensities = old_msg.intensities
		#No matching field name in old message
		new_msg.foo = 0

