//{
        if(index($0,"VERBOSE=1")>0) {
                printf "\t\t\t\t\t<value>VERBOSE=1|"
                printf "ROS_ROOT="ENVIRON["ROS_ROOT"]"|"
                printf "ROS_PACKAGE_PATH="ENVIRON["ROS_PACKAGE_PATH"]"|"
                printf "PYTHONPATH="ENVIRON["PYTHONPATH"]"|"
                printf "PATH="ENVIRON["PATH"]"|"
                printf "LD_LIBRARY_PATH="ENVIRON["LD_LIBRARY_PATH"]"|"
                printf "PKG_CONFIG_PATH="ENVIRON["PKG_CONFIG_PATH"]"|"
                printf "CMAKE_PREFIX_PATH="ENVIRON["CMAKE_PREFIX_PATH"]"|"
                print "</value>"
        } else {
                print $0
        }
}
