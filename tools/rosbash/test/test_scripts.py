#!/usr/bin/env python

import os
import subprocess
import unittest

PKG_PATH = os.getcwd()
TEST_PATH = os.path.join(PKG_PATH, 'test')

class TestRosBash(unittest.TestCase):

    def setUp(self):
        self.cmdbash = os.path.join(TEST_PATH, 'test_rosbash.bash')
        self.assertTrue(os.path.exists(self.cmdbash))
        
        self.cmdzsh = os.path.join(TEST_PATH, 'test_roszsh.zsh')
        self.assertTrue(os.path.exists(self.cmdzsh))
        
    def test_rosbash_completion(self):
        subprocess.check_call([self.cmdbash], cwd = TEST_PATH)
 
    def test_roszsh_completion(self):
        subprocess.check_call([self.cmdzsh], cwd = TEST_PATH)
