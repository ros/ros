rospack_add_lisp_executable(bin/add-two-ints-client src/add-two-ints-client.lisp)
# Disabled, because it uses std_msgs/ChannelFloat32, which no longer exists
#rospack_add_lisp_executable(bin/array-listener src/array-listener.lisp)
#rospack_add_lisp_executable(bin/array-talker src/array-talker.lisp)
rospack_add_lisp_executable(bin/listener src/listener.lisp)
rospack_add_lisp_executable(bin/add-two-ints-server src/add-two-ints-server.lisp)
rospack_add_lisp_executable(bin/talker src/talker.lisp)

