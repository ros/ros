#!/usr/bin/env python

import os
import shutil
import subprocess
import tempfile
import unittest

PKG_PATH = os.getcwd()
TEST_PATH = os.path.join(PKG_PATH, 'test')


def make_bash_pre_command(strings, currentword):
    return "bash -c '. %s; export COMP_WORDS=(%s); export COMP_CWORD=%s;" % (os.path.join(PKG_PATH, 'rosbash'), ' '.join(['"%s"' % w for w in strings]), currentword)


class TestRosBash(unittest.TestCase):

    def setUp(self):
        self.cmdbash = os.path.join(TEST_PATH, 'test_rosbash.bash')
        self.assertTrue(os.path.exists(self.cmdbash))

        self.cmdzsh = os.path.join(TEST_PATH, 'test_roszsh.zsh')
        self.assertTrue(os.path.exists(self.cmdzsh))

    def test_rosbash_completion(self):
        subprocess.check_call([self.cmdbash], cwd=TEST_PATH)

    def test_roszsh_completion(self):
        subprocess.check_call([self.cmdzsh], cwd=TEST_PATH)


class TestWithFiles(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        self.test_root_path = tempfile.mkdtemp()

    @classmethod
    def tearDownClass(self):
        shutil.rmtree(self.test_root_path)

    def test_make_precommand(self):
        self.assertEqual("bash -c '. %s; export COMP_WORDS=(\"foo\" \"bar\"); export COMP_CWORD=1;" % os.path.join(PKG_PATH, 'rosbash'), make_bash_pre_command(['foo', 'bar'], 1))
        self.assertEqual("bash -c '. %s; export COMP_WORDS=(\"foo\"); export COMP_CWORD=2;" % os.path.join(PKG_PATH, 'rosbash'), make_bash_pre_command(['foo'], 2))

    def test_roslaunch_completion(self):
        # regression test that roslaunch completion works even in the presence of launchfiles
        subprocess.check_call('touch foo.launch', shell=True, cwd=self.test_root_path)
        subprocess.check_call('touch bar.launch', shell=True, cwd=self.test_root_path)

        cmd = make_bash_pre_command(['rosbash', 'rosbash'], 2)
        cmd += "_roscomplete_launch rosbash rosbash; echo $COMPREPLY'"
        p = subprocess.Popen(cmd,
                             shell=True,
                             stdout=subprocess.PIPE,
                             cwd=self.test_root_path)
        output = p.communicate()
        self.assertEqual(0, p.returncode, (p.returncode, output, cmd))

        self.assertTrue('example.launch' in output[0], (p.returncode, output[0], cmd))
