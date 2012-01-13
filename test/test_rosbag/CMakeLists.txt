project(test_rosbag)
catkin_project(test_rosbag)

add_pyunit(test/migrate_test.py TIMEOUT 120.0)
add_rostest(test/random_record.xml)
add_rostest(test/random_play.xml)
add_rostest(test/random_play_sim.xml)

