# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

"""
Dumps parameters from ROSLaunch files to stdout as YAML.
"""

import sys

import roslaunch.config
import roslaunch.xmlloader 

import yaml

def dump_params(files):
    """
    Dumps ROS parameters of a list of files to STDOUT in YAML
    
    @param files: List of ROSLaunch files to load
    @type  files: [ str ]
    @return: True if loaded parameters successfully
    @rtype: bool
    """
    config = roslaunch.config.ROSLaunchConfig()
    loader = roslaunch.xmlloader.XmlLoader()

    for f in files:
        try:
            loader.load(f, config, verbose = False)
        except Exception as e:
            sys.stderr.write("Unable to load file %s: %s" % (f, e))
            return False

    # Now print params in YAML format.
    params_dict = {}
    for k, v in config.params.iteritems():
        params_dict[str(k)] = v.value
    sys.stdout.write(yaml.safe_dump(params_dict)+'\n')
    return True
