#! /usr/bin/env python

import re

class Warnings:
    """ Extract warnings from GCC's output

    Analyzes compiler output and classifies warnings.
    """

    _warning_pattern_map = {
        'antiquated':' antiquated',
        'deprecated' : ' deprecated',
        'unused_func' : ' defined but not used',
        'isoc' : ' ISO C',
        'missing_init' : ' missing initializer',
        'out_of_bounds' : ' subscript .*? bounds',
        'unused_var' : ' unused variable'
        }

    def __init__(self, console_output):
        self.warning_lines = [ x for x in console_output.splitlines() if x.find(" warning:") > 0 ]
    
    def byType(self, warntype):
        """ Extract warning messages corresponding to warntype.
        The warntypes can be all keys of the _warning_pattern_map dictionary.
        @param warntype: The type of warning message that should be extracted.
        @type warntype: str
        @return a list of warning messages
        @rtype list
        """
        return [ x for x in self.warning_lines if re.search(self._warning_pattern_map[warntype], x) ]
    
    def analyze(self):
        """ Get dictionary of classified warnings.

        @return A dictionary of lists of warning messages indexed by the warning type
        @rtype {str:[str]}
        """
        return dict( [ (t,self.byType(t)) for t,p in self._warning_pattern_map.iteritems() ] )
