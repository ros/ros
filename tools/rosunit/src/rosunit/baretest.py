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
# Revision $Id$

"""
rostest implementation of running bare (gtest-compatible) unit test
executables. These do not run in a ROS environment.
"""

from __future__ import print_function

import errno
import os
try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO
import signal
import subprocess
import time
import traceback
import unittest

import rospkg

from . import junitxml
from . import pmon
from .core import create_xml_runner  # noqa: F401
from .core import printerrlog
from .core import printlog
from .core import printlog_bold
from .core import rostest_name_from_path  # noqa: F401
from .core import xml_results_file

BARE_TIME_LIMIT = 60.
TIMEOUT_SIGINT = 15.0  # seconds
TIMEOUT_SIGTERM = 2.0  # seconds


class TestTimeoutException(Exception):
    pass


class BareTestCase(unittest.TestCase):

    def __init__(self, exe, args, retry=0, time_limit=None, test_name=None, text_mode=False, package_name=None):
        """
        @param exe: path to executable to run
        @type  exe: str
        @param args: arguments to exe
        @type  args: [str]
        @type  retry: int
        @param time_limit: (optional) time limit for test. Defaults to BARE_TIME_LIMIT.
        @type  time_limit: float
        @param test_name: (optional) override automatically generated test name
        @type  test_name: str
        @param package_name: (optional) override automatically inferred package name
        @type  package_name: str
        """
        super(BareTestCase, self).__init__()
        self.text_mode = text_mode
        if package_name:
            self.package = package_name
        else:
            self.package = rospkg.get_package_name(exe)
        self.exe = os.path.abspath(exe)
        if test_name is None:
            self.test_name = os.path.basename(exe)
        else:
            self.test_name = test_name

        # invoke pyunit tests with python executable
        if self.exe.endswith('.py'):
            self.args = ['python', self.exe] + args
        else:
            self.args = [self.exe] + args
        if text_mode:
            self.args = self.args + ['--text']

        self.retry = retry
        self.time_limit = time_limit or BARE_TIME_LIMIT
        self.pmon = None
        self.results = junitxml.Result(self.test_name)

    def setUp(self):
        self.pmon = pmon.start_process_monitor()

    def tearDown(self):
        if self.pmon is not None:
            pmon.shutdown_process_monitor(self.pmon)
            self.pmon = None

    def runTest(self):
        self.failIf(self.package is None, 'unable to determine package of executable')

        done = False
        while not done:
            test_name = self.test_name

            printlog('Running test [%s]', test_name)

            # setup the test
            # - we pass in the output test_file name so we can scrape it
            test_file = xml_results_file(self.package, test_name, False)
            if os.path.exists(test_file):
                printlog('removing previous test results file [%s]', test_file)
                os.remove(test_file)

            self.args.append('--gtest_output=xml:%s' % test_file)

            # run the test, blocks until completion
            printlog('running test %s' % test_name)
            timeout_failure = False

            run_id = None
            # TODO: really need different, non-node version of LocalProcess instead of these extra args
            process = LocalProcess(run_id, self.package, self.test_name, self.args, os.environ, False, cwd='cwd', is_node=False)

            pm = self.pmon
            pm.register(process)
            success = process.start()
            self.assert_(success, 'test failed to start')

            # poll until test terminates or alloted time exceed
            timeout_t = time.time() + self.time_limit
            try:
                while process.is_alive():
                    # test fails on timeout
                    if time.time() > timeout_t:
                        raise TestTimeoutException('test max time allotted')
                    time.sleep(0.1)

            except TestTimeoutException:
                if self.retry:
                    timeout_failure = True
                else:
                    raise

            if not timeout_failure:
                printlog('test [%s] finished' % test_name)
            else:
                printerrlog('test [%s] timed out' % test_name)

            if self.text_mode:
                results = self.results
            elif not self.text_mode:
                # load in test_file
                if not timeout_failure:
                    self.assert_(os.path.isfile(test_file), 'test [%s] did not generate test results' % test_name)
                    printlog('test [%s] results are in [%s]', test_name, test_file)
                    results = junitxml.read(test_file, test_name)
                    test_fail = results.num_errors or results.num_failures
                else:
                    test_fail = True

            if self.retry > 0 and test_fail:
                self.retry -= 1
                printlog('test [%s] failed, retrying. Retries left: %s' % (test_name, self.retry))
            else:
                done = True
                self.results = results
                printlog('test [%s] results summary: %s errors, %s failures, %s tests',
                         test_name, results.num_errors, results.num_failures, results.num_tests)

        printlog('[ROSTEST] test [%s] done', test_name)


# TODO: this is a straight copy from roslaunch. Need to reduce, refactor
class LocalProcess(pmon.Process):
    """
    Process launched on local machine
    """

    def __init__(self, run_id, package, name, args, env, log_output, respawn=False, required=False, cwd=None, is_node=True):
        """
        @param run_id: unique run ID for this roslaunch. Used to
          generate log directory location. run_id may be None if this
          feature is not being used.
        @type  run_id: str
        @param package: name of package process is part of
        @type  package: str
        @param name: name of process
        @type  name: str
        @param args: list of arguments to process
        @type  args: [str]
        @param env: environment dictionary for process
        @type  env: {str : str}
        @param log_output: if True, log output streams of process
        @type  log_output: bool
        @param respawn: respawn process if it dies (default is False)
        @type  respawn: bool
        @param cwd: working directory of process, or None
        @type  cwd: str
        @param is_node: (optional) if True, process is ROS node and accepts ROS node command-line arguments. Default: True
        @type  is_node: False
        """
        super(LocalProcess, self).__init__(package, name, args, env, respawn, required)
        self.run_id = run_id
        self.popen = None
        self.log_output = log_output
        self.started = False
        self.stopped = False
        self.cwd = cwd
        self.log_dir = None
        self.pid = -1
        self.is_node = is_node

    # NOTE: in the future, info() is going to have to be sufficient for relaunching a process
    def get_info(self):
        """
        Get all data about this process in dictionary form
        """
        info = super(LocalProcess, self).get_info()
        info['pid'] = self.pid
        if self.run_id:
            info['run_id'] = self.run_id
        info['log_output'] = self.log_output
        if self.cwd is not None:
            info['cwd'] = self.cwd
        return info

    def _configure_logging(self):
        """
        Configure logging of node's log file and stdout/stderr
        @return: stdout log file name, stderr log file
        name. Values are None if stdout/stderr are not logged.
        @rtype: str, str
        """
        log_dir = rospkg.get_log_dir(env=os.environ)
        if self.run_id:
            log_dir = os.path.join(log_dir, self.run_id)
        if not os.path.exists(log_dir):
            try:
                os.makedirs(log_dir)
            except OSError as e:
                if e.errno == errno.EACCES:
                    raise RLException('unable to create directory for log file [%s].\nPlease check permissions.' % log_dir)
                else:
                    raise RLException('unable to create directory for log file [%s]: %s' % (log_dir, e.strerror))
        # #973: save log dir for error messages
        self.log_dir = log_dir

        # send stdout/stderr to file. in the case of respawning, we have to
        # open in append mode
        # note: logfileerr: disabling in favor of stderr appearing in the console.
        # will likely reinstate once roserr/rosout is more properly used.
        logfileout = logfileerr = None

        if self.log_output:
            outf, errf = [os.path.join(log_dir, '%s-%s.log' % (self.name, n)) for n in ['stdout', 'stderr']]
            if self.respawn:
                mode = 'a'
            else:
                mode = 'w'
            logfileout = open(outf, mode)
            if is_child_mode():
                logfileerr = open(errf, mode)

        # #986: pass in logfile name to node
        if self.is_node:
            # #1595: on respawn, these keep appending
            self.args = _cleanup_remappings(self.args, '__log:=')
            self.args.append('__log:=%s' % os.path.join(log_dir, '%s.log' % self.name))

        return logfileout, logfileerr

    def start(self):
        """
        Start the process.

        @raise pmon.FatalProcessLaunch: if process cannot be started and it
        is not likely to ever succeed
        """
        super(LocalProcess, self).start()
        try:
            self.lock.acquire()
            self.started = self.stopped = False

            full_env = self.env

            # _configure_logging() can mutate self.args
            try:
                logfileout, logfileerr = self._configure_logging()
            except Exception as e:
                printerrlog('[%s] ERROR: unable to configure logging [%s]' % (self.name, str(e)))
                # it's not safe to inherit from this process as
                # rostest changes stdout to a StringIO, which is not a
                # proper file.
                logfileout, logfileerr = subprocess.PIPE, subprocess.PIPE

            if self.cwd == 'node':
                cwd = os.path.dirname(self.args[0])
            elif self.cwd == 'cwd':
                cwd = os.getcwd()
            elif self.cwd == 'ros-root':
                from roslib.rosenv import get_ros_root
                cwd = get_ros_root()
            else:
                cwd = rospkg.get_ros_home()
            if not os.path.exists(cwd):
                try:
                    os.makedirs(cwd)
                except OSError:
                    # exist_ok=True
                    pass

            try:
                self.popen = subprocess.Popen(self.args, cwd=cwd, stdout=logfileout, stderr=logfileerr, env=full_env, close_fds=True, preexec_fn=os.setsid)
            except OSError as e:
                self.started = True  # must set so is_alive state is correct
                if e.errno == errno.ENOEXEC:  # Exec format error
                    raise pmon.FatalProcessLaunch("Unable to launch [%s]. \nIf it is a script, you may be missing a '#!' declaration at the top." % self.name)
                elif e.errno == errno.ENOENT:  # no such file or directory
                    raise pmon.FatalProcessLaunch("""Roslaunch got a '%s' error while attempting to run:

%s

Please make sure that all the executables in this command exist and have
executable permission. This is often caused by a bad launch-prefix.""" % (msg, ' '.join(self.args)))
                else:
                    raise pmon.FatalProcessLaunch('unable to launch [%s]: %s' % (' '.join(self.args), msg))

            self.started = True
            # Check that the process is either still running (poll returns
            # None) or that it completed successfully since when we
            # launched it above (poll returns the return code, 0).
            poll_result = self.popen.poll()
            if poll_result is None or poll_result == 0:
                self.pid = self.popen.pid
                printlog_bold('process[%s]: started with pid [%s]' % (self.name, self.pid))
                return True
            else:
                printerrlog('failed to start local process: %s' % (' '.join(self.args)))
                return False
        finally:
            self.lock.release()

    def is_alive(self):
        """
        @return: True if process is still running
        @rtype: bool
        """
        if not self.started:  # not started yet
            return True
        if self.stopped or self.popen is None:
            return False
        self.exit_code = self.popen.poll()
        if self.exit_code is not None:
            return False
        return True

    def get_exit_description(self):
        """
        @return: human-readable description of exit state
        @rtype: str
        """
        # #973: include location of output location in message
        if self.exit_code is not None:
            if self.exit_code:
                if self.log_dir:
                    return 'process has died [pid %s, exit code %s].\nlog files: %s*.log' % (self.pid, self.exit_code, os.path.join(self.log_dir, self.name))
                else:
                    return 'process has died [pid %s, exit code %s]' % (self.pid, self.exit_code)
            else:
                if self.log_dir:
                    return 'process has finished cleanly.\nlog file: %s*.log' % (os.path.join(self.log_dir, self.name))
                else:
                    return 'process has finished cleanly'
        else:
            return 'process has died'

    def _stop_unix(self, errors):
        """
        UNIX implementation of process killing

        @param errors: error messages. stop() will record messages into this list.
        @type  errors: [str]
        """
        self.exit_code = self.popen.poll()
        if self.exit_code is not None:
            # print "process[%s].stop(): process has already returned %s"%(self.name, self.exit_code)
            self.popen = None
            self.stopped = True
            return

        pid = self.popen.pid
        pgid = os.getpgid(pid)

        try:
            # Start with SIGINT and escalate from there.
            os.killpg(pgid, signal.SIGINT)
            timeout_t = time.time() + TIMEOUT_SIGINT
            retcode = self.popen.poll()
            while time.time() < timeout_t and retcode is None:
                time.sleep(0.1)
                retcode = self.popen.poll()
            # Escalate non-responsive process
            if retcode is None:
                printerrlog('[%s] escalating to SIGTERM' % self.name)
                timeout_t = time.time() + TIMEOUT_SIGTERM
                os.killpg(pgid, signal.SIGTERM)
                retcode = self.popen.poll()
                while time.time() < timeout_t and retcode is None:
                    time.sleep(0.2)
                    retcode = self.popen.poll()
                if retcode is None:
                    printerrlog('[%s] escalating to SIGKILL' % self.name)
                    errors.append('process[%s, pid %s]: required SIGKILL. May still be running.' % (self.name, pid))
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                        # #2096: don't block on SIGKILL, because this results in more orphaned processes overall
                    except OSError as e:
                        if e.args[0] == 3:
                            printerrlog('no [%s] process with pid [%s]' % (self.name, pid))
                        else:
                            printerrlog('errors shutting down [%s]: %s' % (self.name, e))
        finally:
            self.popen = None

    def stop(self, errors=[]):
        """
        Stop the process. Record any significant error messages in the errors parameter

        @param errors: error messages. stop() will record messages into this list.
        @type  errors: [str]
        """
        super(LocalProcess, self).stop(errors)
        self.lock.acquire()
        try:
            try:
                if self.popen is None:
                    return
                # NOTE: currently POSIX-only. Need to add in Windows code once I have a test environment:
                # http://aspn.activestate.com/ASPN/Cookbook/Python/Recipe/347462
                self._stop_unix(errors)
            except Exception:
                printerrlog('[%s] EXCEPTION %s' % (self.name, traceback.format_exc()))
        finally:
            self.stopped = True
            self.lock.release()


def print_runner_summary(runner_results, junit_results, runner_name='ROSUNIT'):
    """
    Print summary of runner results and actual test results to
    stdout. For rosunit and rostest, the test is wrapped in an
    external runner. The results from this runner are important if the
    runner itself has a failure.

    @param runner_result: unittest runner result object
    @type  runner_result: _XMLTestResult
    @param junit_results: Parsed JUnit test results
    @type  junit_results: rosunit.junitxml.Result
    """
    # we have two separate result objects, which can be a bit
    # confusing. 'result' counts successful _running_ of tests
    # (i.e. doesn't check for actual test success). The 'r' result
    # object contains results of the actual tests.

    buff = StringIO()

    buff.write('[%s]' % (runner_name) + '-' * 71 + '\n\n')
    for tc_result in junit_results.test_case_results:
        buff.write(tc_result.description)
    for tc_result in runner_results.failures:
        buff.write('[%s][failed]\n' % tc_result[0]._testMethodName)

    buff.write('\nSUMMARY\n')
    if runner_results.wasSuccessful() and (junit_results.num_errors + junit_results.num_failures) == 0:
        buff.write('\033[32m * RESULT: SUCCESS\033[0m\n')
    else:
        buff.write('\033[1;31m * RESULT: FAIL\033[0m\n')

    # TODO: still some issues with the numbers adding up if tests fail to launch

    # number of errors from the inner tests, plus add in count for tests
    # that didn't run properly ('result' object).
    buff.write(' * TESTS: %s\n' % junit_results.num_tests)
    num_errors = junit_results.num_errors+len(runner_results.errors)
    if num_errors:
        buff.write('\033[1;31m * ERRORS: %s\033[0m\n' % num_errors)
    else:
        buff.write(' * ERRORS: 0\n')
    num_failures = junit_results.num_failures+len(runner_results.failures)
    if num_failures:
        buff.write('\033[1;31m * FAILURES: %s\033[0m\n' % num_failures)
    else:
        buff.write(' * FAILURES: 0\n')

    if runner_results.failures:
        buff.write('\nERROR: The following tests failed to run:\n')
        for tc_result in runner_results.failures:
            buff.write(' * ' + tc_result[0]._testMethodName + '\n')

    print(buff.getvalue())


def _format_errors(errors):
    formatted = []
    for e in errors:
        if '_testMethodName' in e[0].__dict__:
            formatted.append(e[0]._testMethodName)
        elif 'description' in e[0].__dict__:
            formatted.append('%s: %s\n' % (str(e[0].description), str(e[1])))
        else:
            formatted.append(str(e[0].__dict__))
    return formatted


def print_unittest_summary(result):
    """
    Print summary of python unittest result to stdout
    @param result: test results
    """
    buff = StringIO()
    buff.write('-------------------------------------------------------------\nSUMMARY:\n')
    if result.wasSuccessful():
        buff.write('\033[32m * RESULT: SUCCESS\033[0m\n')
    else:
        buff.write(' * RESULT: FAIL\n')
    buff.write(' * TESTS: %s\n' % result.testsRun)
    buff.write(' * ERRORS: %s [%s]\n' % (len(result.errors), ', '.join(_format_errors(result.errors))))
    buff.write(' * FAILURES: %s [%s]\n' % (len(result.failures), ', '.join(_format_errors(result.failures))))
    print(buff.getvalue())
