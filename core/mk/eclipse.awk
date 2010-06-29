//{
        if(index($0,"VERBOSE=1")>0) {
                printf "\t\t\t\t\t<value>VERBOSE=1|"
                printf "ROS_ROOT="ENVIRON["ROS_ROOT"]"|"
                printf "ROS_PACKAGE_PATH="ENVIRON["ROS_PACKAGE_PATH"]"|"
                printf "PYTHONPATH="ENVIRON["PYTHONPATH"]"|"
                printf "PATH="ENVIRON["PATH"]"|"
                print "</value>"
        } else {
                print $0
        }
}

