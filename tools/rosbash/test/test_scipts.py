#!/usr/bin/env python

import os
import subprocess
import unittest

PKG_PATH = os.getcwd()
TEST_PATH = os.path.join(PKG_PATH, 'test')

class TestRosBash(unittest.TestCase):

    def setUp(self):
        self.cmd = os.path.join(TEST_PATH, 'test_rosbash.bash')
        self.assertTrue(os.path.exists(self.cmd))
    
    def test_rosbash_completion(self):
        subprocess.check_call([self.cmd], cwd = TEST_PATH)
 
