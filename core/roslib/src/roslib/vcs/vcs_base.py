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
vcs support library base class.

New in ROS C-Turtle.
"""
import os

class VCSClientBase:
    def __init__(self, path):
        self._path = path
        
    def path_exists(self):
        return os.path.exists(self._path)
        
    def get_path(self):
        return self._path

    def get_url(self):
        """
        @return: The source control url for the path
        @rtype: str
        """
        raise NotImplementedError, "Base class get_url method must be overridden"

    def get_version(self):
        raise NotImplementedError, "Base class get_version method must be overridden"

    def checkout(self, url, version):
        raise NotImplementedError, "Base class checkout method must be overridden"

    def update(self, version):
        raise NotImplementedError, "Base class update method must be overridden"
    

    def detect_presence(self):
        """For auto detection"""
        raise NotImplementedError, "Base class detect_presence method must be overridden"

    def get_vcs_type_name(self):
        """ used when auto detected """
        raise NotImplementedError, "Base class get_vcs_type_name method must be overridden"
