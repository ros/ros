from __future__ import print_function

import os
import subprocess
import sys

ROS_CACHE_TIMEOUT_ENV_NAME = 'ROS_CACHE_TIMEOUT'
ROS_LOCATIONS_ENV_NAME = 'ROS_LOCATIONS'
ROS_LOCATION_SEP = ';'
ERROR_PREFIX = 'Error: '


def ros_location_find(package_name):
    # process ROS_LOCATION and return if found
    ros_location = os.environ.get(ROS_LOCATIONS_ENV_NAME)
    if ros_location is not None:
        locations = ros_location.split(ROS_LOCATION_SEP)
        for loc in locations:
            index = loc.find('=')
            if index != -1:
                if package_name == loc[:index]:
                    return 0, loc[index + 1:]

    if package_name == 'log':
        p = subprocess.Popen('roslaunch-logs', stdout=subprocess.PIPE)
        result_location = p.communicate()[0].strip()
        result_code = p.returncode
        return result_code, result_location if result_code == 0 else ''

    if package_name == 'test_results':
        p = subprocess.Popen('rosrun.bat rosunit test_results_dir.py', stdout=subprocess.PIPE)
        result_location = p.communicate()[0].strip()
        result_code = p.returncode
        return result_code, result_location if result_code == 0 else ''

    # process package_name and return
    env = os.environ
    env[ROS_CACHE_TIMEOUT_ENV_NAME] = '-1.0'
    p = subprocess.Popen(['rospack', 'find', package_name], stdout=subprocess.PIPE)
    result_location = p.communicate()[0].strip()
    result_code = p.returncode
    if result_code == 0:
        return result_code, result_location

    p = subprocess.Popen(['rosstack', 'find', package_name], stdout=subprocess.PIPE)
    result_location = p.communicate()[0].strip()
    result_code = p.returncode
    if result_code == 0:
        return result_code, result_location

    # package <package_name> not found
    return result_code, ''


# takes as argument either just a package-path or just a pkgname
# returns 0 for no argument or if package (+ path) exist, 1 else
# on success with arguments print result_path or Error: error message
def findpathmain(argv):
    reldir = ''
    parameters = os.path.normpath(argv[0]).split(os.path.sep)
    package_name = parameters[0]
    if len(parameters) > 1:
        reldir = os.path.sep.join(parameters[1:])
    else:
        if len(argv) < 2 or argv[1] != 'forceeval':
            print(ERROR_PREFIX + '[' + package_name + '] is not a valid argument!', file=sys.stderr)
            return 1

    error_code, package_dir = ros_location_find(package_name)
    if error_code != 0:
        print(ERROR_PREFIX + '[' + package_name + '] not found!', file=sys.stderr)
        return error_code
    else:
        rosdir = os.path.normpath(os.path.sep.join([package_dir, reldir]))
        print(rosdir)
        return 0


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(1)
    sys.exit(findpathmain(sys.argv[1:]))
