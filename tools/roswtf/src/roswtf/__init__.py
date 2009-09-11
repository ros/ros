# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
# Revision $Id$

def roswtf_main():
    from roswtf.context import WtfContext
    from roswtf.environment import wtf_check_environment
    from roswtf.graph import wtf_check_graph
    from roswtf.packages import wtf_check_packages
    from roswtf.roslaunch import wtf_check_roslaunch
    
    print "roswtf demo"
    ctx = WtfContext.from_package('roswtf')
    #TODO: load roslaunch files if roslaunch file specified
    
    print "context loaded"    
    print "checking environment"    
    wtf_check_environment(ctx)
    wtf_check_packages(ctx)
    # TODO: break into static and online checks so we can delay time-consuming work until later
    wtf_check_roslaunch(ctx)    
    wtf_check_graph(ctx)        

    print "environment checked"
    print "Found %s warning(s)"%len(ctx.warnings)

    for warn in ctx.warnings:
        print "WARNING:", warn.msg
    print "Found %s error(s)"%len(ctx.errors)        
    for warn in ctx.errors:
        print "ERROR:", warn.msg
    
