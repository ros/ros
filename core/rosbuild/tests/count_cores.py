#!/usr/bin/env python

import os
print(os.sysconf(os.sysconf_names['SC_NPROCESSORS_ONLN']))
