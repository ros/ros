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

import sys
import time

class ProgressMeter:
    SUFFIXES = {1000: ['KB', 'MB', 'GB', 'TB', 'PB', 'EB', 'ZB', 'YB'],
                1024: ['KiB', 'MiB', 'GiB', 'TiB', 'PiB', 'EiB', 'ZiB', 'YiB']}

    def __init__(self, path, bytes_total, refresh_rate=1.0):
        self.path           = path
        self.bytes_total    = bytes_total
        self.refresh_rate   = refresh_rate
        
        self.elapsed        = 0.0
        self.update_elapsed = 0.0
        self.bytes_read     = 0.0

        self.start_time     = time.time()

        self._update_progress()

    def step(self, bytes_read, force_update=False):
        self.bytes_read = bytes_read
        self.elapsed    = time.time() - self.start_time
        
        if force_update or self.elapsed - self.update_elapsed > self.refresh_rate:
            self._update_progress()
            self.update_elapsed = self.elapsed

    def _update_progress(self):
        max_path_len = self.terminal_width() - 36
        path = self.path
        if len(path) > max_path_len:
            path = '...' + self.path[-max_path_len + 3:]

        bytes_read_str  = self.approximate_size(float(self.bytes_read))
        bytes_total_str = self.approximate_size(float(self.bytes_total))
        
        if self.bytes_read < self.bytes_total:
            complete_fraction = float(self.bytes_read) / self.bytes_total
            pct_complete      = int(100.0 * complete_fraction)

            if complete_fraction > 0.0:
                eta = self.elapsed * (1.0 / complete_fraction - 1.0)
                eta_min, eta_sec = eta / 60, eta % 60
                if eta_min > 99:
                    eta_str = '--:--'
                else:
                    eta_str = '%02d:%02d' % (eta_min, eta_sec)
            else:
                eta_str = '--:--'

            progress = '%-*s %3d%% %8s / %8s %s ETA' % (max_path_len, path, pct_complete, bytes_read_str, bytes_total_str, eta_str)
        else:
            progress = '%-*s 100%% %19s %02d:%02d    ' % (max_path_len, path, bytes_total_str, self.elapsed / 60, self.elapsed % 60)

        print '\r', progress,
        sys.stdout.flush()

    def finish(self):
        self.step(self.bytes_total, force_update=True)
        print

    ## Convert file size to human-readable form
    @classmethod
    def approximate_size(cls, size, a_kilobyte_is_1024_bytes=False):
        if size < 0:
            raise ValueError('number must be non-negative')

        multiple = 1024 if a_kilobyte_is_1024_bytes else 1000
        for suffix in cls.SUFFIXES[multiple]:
            size /= multiple
            if size < multiple:
                return '%.1f %s' % (size, suffix)
    
        raise ValueError('number too large')

    ## Estimate the width of the terminal
    @staticmethod
    def terminal_width():
        width = 0
        try:
            import struct, fcntl, termios
            s     = struct.pack('HHHH', 0, 0, 0, 0)
            x     = fcntl.ioctl(1, termios.TIOCGWINSZ, s)
            width = struct.unpack('HHHH', x)[1]
        except IOError:
            pass
        if width <= 0:
            try:
                width = int(os.environ['COLUMNS'])
            except:
                pass
        if width <= 0:
            width = 80
    
        return width
