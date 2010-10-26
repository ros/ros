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

"""
Common library for writing rule-style checks for generating warnings
and errors.  Use of this style streamlines reporting.

The pattern for rules is simple: a rule provides a function that
implements the rule and a format string. If the function returns a
non-zero value, that value is combined with the format string to
produced an error reporting string. There are other conveniences as
well. If the rule returns a list or a tuple, that will be transformed
into a human-readable list.

This library is a layer on top of the base L{WtfWarning} and
L{WtfError} representation in roswtf.model.
"""

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
    
def warning_rule(rule, ret, ctx):
    """
    Check return value of rule and update ctx if rule failed.
    
    @param rule: Rule/message pair.
    @type  rule: (rule_fn, format_msg)
    @param ret: return value of rule. If value is non-zero, rule failed
    @param ret: Any
    @param ctx: context for which rule failed
    @param ctx: L{WtfContext}
    """
    _check_rule(rule, ret, ctx, ctx.warnings, WtfWarning)
    
def error_rule(rule, ret, ctx):
    """
    Check return value of rule and update ctx if rule failed.
    
    @param rule: Rule/message pair.
    @type  rule: (rule_fn, format_msg)
    @param ret: return value of rule. If value is non-zero, rule failed
    @type  ret: Any
    @param ctx: context for which rule failed
    @type  ctx: L{WtfContext}
    """
    _check_rule(rule, ret, ctx, ctx.errors, WtfError)
    
