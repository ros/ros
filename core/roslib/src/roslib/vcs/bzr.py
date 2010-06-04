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
bzr vcs support.

New in ROS C-Turtle.
"""

import subprocess
import os
import vcs_base

class BZRClient(vcs_base.VCSClientBase):
    def get_url(self):
        """
        @return: BZR URL of the directory path (output of bzr info command), or None if it cannot be determined
        """
        if self.detect_presence():
            output = subprocess.Popen(['bzr', 'info', self._path], stdout=subprocess.PIPE).communicate()[0]
            matches = [l for l in output.split('\n') if l.startswith('  parent branch:')]
            if matches:
                return matches[0][17:]
        return None

    def detect_presence(self):
        return self.path_exists() and os.path.isdir(os.path.join(self._path, '.bzr'))


    def checkout(self, url, version=''):
        if self.path_exists():
            print >>sys.stderr, "Error: cannnot checkout into existing directory"
            return False
            
        cmd = "bzr branch %s %s %s"%(version, url, self._path)
        if subprocess.check_call(cmd, shell=True) == 0:
            return True
        return False

    def update(self, version=''):
        if not self.detect_presence():
            return False
        if not subprocess.check_call("bzr pull", cwd=self._path, shell=True) == 0:
            return False
        cmd = "bzr uncommit %s --force"%(version)
        if subprocess.check_call(cmd, cwd=self._path, shell=True) == 0:
            return True
        return False
        
    def get_vcs_type_name(self):
        return 'bzr'


    def get_version(self):
        output = subprocess.Popen(['bzr', 'revno'], cwd= self._path, stdout=subprocess.PIPE).communicate()[0]
        return "-r"+output.strip()
