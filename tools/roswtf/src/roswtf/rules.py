# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

from roswtf.model import WtfWarning, WtfError

def _check_rule(rule, ret, ctx, ctx_list, level):
    if ret:
        d = ctx.as_dictionary()

        if type(ret) in (tuple, list):
            f_msg = rule[1]
            ret_str = '\n'.join([" * %s"%r for r in ret])
            ctx_list.append(level(f_msg%d + "\n" + ret_str+'\n', f_msg, ret))
        elif isinstance(ret, basestring):
            f_msg = rule[1]
            ctx_list.append(level(f_msg%d + ret%d, f_msg, ret))
        else:
            f_msg = rule[1]
            ctx_list.append(level(f_msg%d, f_msg, ret))
    
## Check return value of rule and update \a ctx if rule failed
## @param rule (rule_fn, format_msg)
## @param ret Any: return value of rule. If value is non-zero, rule failed
## @param ctx WtfContext: context for which rule failed
def warning_rule(rule, ret, ctx):
    _check_rule(rule, ret, ctx, ctx.warnings, WtfWarning)
    
## Check return value of rule and update \a ctx if rule failed
## @param rule (rule_fn, format_msg)
## @param ret Any: return value of rule. If value is non-zero, rule failed
## @param ctx WtfContext: context for which rule failed
def error_rule(rule, ret, ctx):
    _check_rule(rule, ret, ctx, ctx.errors, WtfError)
    
