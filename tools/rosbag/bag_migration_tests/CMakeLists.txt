# This is going to need some work because the bag files on
# pr.willowgarage.com all have messages with the test_rosbag package
# name.  Of course the package name for this package now is rosbag.
# If we can have a way to pass an override package name into
# generate_messages() it might make it work, or we can migrate the bag
# files themselves.

add_message_files(
  DIRECTORY msg_current
  FILES
  Constants.msg
  Converged.msg
  MigratedAddSub.msg
  MigratedExplicit.msg
  MigratedImplicit.msg
  MigratedMixed.msg
  PartiallyMigrated.msg
  Renamed4.msg
  Simple.msg
  SimpleMigrated.msg
  SubUnmigrated.msg
  Unmigrated.msg
)
generate_messages(DEPENDENCIES rosgraph_msgs std_msgs)

download_test_data(http://pr.willowgarage.com/data/test_rosbag/constants_gen1.bag test/constants_gen1.bag 77ec8cb20e823ee3f3a87d07ea1132df )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/constants_gen2.bag test/constants_gen2.bag 4efd8459bf9403d1171dff06abcd6548 )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/converged_gen1.bag test/converged_gen1.bag 8e3524157d31b5761ac951fe16e03e12 )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/converged_gen2.bag test/converged_gen2.bag 0ad4041d2e3bab8262c12020ec3e048e )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/converged_gen3.bag test/converged_gen3.bag 90dd16cea5c51fca65be617654fb6b76 )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/convergent_gen1.bag test/convergent_gen1.bag a840aec95e50f3c828841b688dfab5a2 )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/convergent_gen2.bag test/convergent_gen2.bag fd84c135ba1548985b870d0d5f4113e9 )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/migrated_addsub_gen1.bag test/migrated_addsub_gen1.bag 462e98cef72b5df56ce3feea9ddfc4d2 )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/migrated_explicit_gen1.bag test/migrated_explicit_gen1.bag 0b77e45aec3ddb19a9f9771c945e2615 )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/migrated_explicit_gen2.bag test/migrated_explicit_gen2.bag b237680d6aae1b20355af097cf0f072b )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/migrated_explicit_gen3.bag test/migrated_explicit_gen3.bag 8b883286e23779bbfc30b0e5588a4d64 )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/migrated_implicit_gen1.bag test/migrated_implicit_gen1.bag 229d6cbd7dcec5e87fb2cf597b837243 )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/migrated_implicit_gen2.bag test/migrated_implicit_gen2.bag df60111e5f8034ba6259c4d76d07339b )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/migrated_implicit_gen3.bag test/migrated_implicit_gen3.bag d86f131070822f1673f7644bc28bfccd )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/migrated_mixed_gen1.bag test/migrated_mixed_gen1.bag 2d0be37de2df6940b7bd93561dde7b51 )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/migrated_mixed_gen2.bag test/migrated_mixed_gen2.bag e000b2de3e4c417b0c9fbba9d2e92dc4 )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/migrated_mixed_gen3.bag test/migrated_mixed_gen3.bag 6e1cbb529b7a45386589d00bf3908bdf )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/partially_migrated_gen1.bag test/partially_migrated_gen1.bag dcfcf1e1c1f7ddc5ab723e171273a385 )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/partially_migrated_gen2.bag test/partially_migrated_gen2.bag 7b2d752559a7e3c724d8b2a032daf3c1 )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/partially_migrated_gen3.bag test/partially_migrated_gen3.bag 01161f84591bf956df7dff0a5e3fd2db )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/renamed_gen1.bag test/renamed_gen1.bag 243d6109f3bfd2ad1af1337a776970e5 )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/renamed_gen2.bag test/renamed_gen2.bag 229c79262e42b71f36e316ab1d1ad943 )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/renamed_gen3.bag test/renamed_gen3.bag c9c3d03807de367b533f5e0384b4d134 )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/subunmigrated_gen1.bag test/subunmigrated_gen1.bag c78b85016aa34669e36d1eb5a7b5f2c7 )
download_test_data(http://pr.willowgarage.com/data/test_rosbag/unmigrated_gen1.bag test/unmigrated_gen1.bag 87d8d3a1c9679609533803709d21cea2 )

configure_file(test/migrate_test.py.in
  ${PROJECT_BINARY_DIR}/test/migrate_test.py)
catkin_add_nosetests(${PROJECT_BINARY_DIR}/test/migrate_test.py)
configure_file(test/random_record.xml.in
  ${PROJECT_BINARY_DIR}/test/random_record.xml)
add_rostest(${PROJECT_BINARY_DIR}/test/random_record.xml)
configure_file(test/random_play.xml.in
  ${PROJECT_BINARY_DIR}/test/random_play.xml)
add_rostest(${PROJECT_BINARY_DIR}/test/random_play.xml)
configure_file(test/random_play_sim.xml.in
  ${PROJECT_BINARY_DIR}/test/random_play_sim.xml)
add_rostest(${PROJECT_BINARY_DIR}/test/random_play_sim.xml)
