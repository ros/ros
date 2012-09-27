cmake_minimum_required(VERSION 2.8)
project(rosmaster)

# genmsg is just needed by the tests.
find_package(catkin REQUIRED COMPONENTS genmsg rostest)

catkin_package()

catkin_python_setup()

add_message_files(DIRECTORY test/msg
  FILES
  String.msg
  Arrays.msg
  CompositeA.msg CompositeB.msg Composite.msg
  Embed.msg Floats.msg Simple.msg
  RosmsgA.msg
  RosmsgB.msg
  RosmsgC.msg
  TestArrays.msg
  TestPrimitives.msg
  TestString.msg
  TVals.msg
)
add_service_files(DIRECTORY test/srv
  FILES
  AddTwoInts.srv
  RossrvA.srv
  RossrvB.srv
)

generate_messages()

add_rostest(test/rosmaster.test)
add_rostest(test/paramserver.test)

catkin_add_nosetests(test/nosetests)

install(FILES manifest.xml
  DESTINATION share/rosmaster)
