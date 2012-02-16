project(roslisp)

foreach(dir
    asdf
    load-manifest
    rosbuild
    roslisp-msg-protocol
    scripts
    src
    s-xml
    s-xml-rpc
    utils)
  install(DIRECTORY ${dir}
          DESTINATION "share/roslisp"
          PATTERN ".svn" EXCLUDE)
endforeach()
