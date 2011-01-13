# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Tingfan Wu tingfan@gmail.com

import os
import subprocess
import roslib.os_detect

import rosdep.base_rosdep

###### Cygwin SPECIALIZATION #########################
def port_detect(p):
    try:
        cmd = ['cygcheck', '-c', p]
        pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (std_out, std_err) = pop.communicate()
        return (std_out.count("OK") > 0)
    except:
        return False

class Cygwin(roslib.os_detect.Cygwin, rosdep.base_rosdep.RosdepBaseOS):
    def strip_detected_packages(self, packages):
        return [p for p in packages if not port_detect(p)] 

    def generate_package_install_command(self, packages, default_yes):        
        return "#Packages\napt-cyg -m ftp://sourceware.org/pub/cygwinports install " + ' '.join(packages)

###### END Cygwin SPECIALIZATION ########################

if __name__ == '__main__':
    print("cygwin installed? %s"%(Cygwin().check_presence()))
    print("test port_detect(true) %s"%(port_detect('cygwin')))
    print("version %s"%(Cygwin().get_version()))
