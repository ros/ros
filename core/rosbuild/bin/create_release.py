#!/usr/bin/env python

# source code refers to 'stacks', but this can work with apps as well
from __future__ import with_statement
NAME="create_release.py"

import sys
import os
from subprocess import Popen, PIPE, call, check_call

import roslib.stacks

try:
    import yaml
except ImportError:
    print >> sys.stderr, "python-yaml not installed, cannot proceed"
    sys.exit(1)
    
class ReleaseException(Exception): pass

def load_sys_args():
    """
    @return: name, version, distro_file
    @rtype: (str, str, str)
    """
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog stack-or-app version distro.yaml", prog=NAME)
    options, args = parser.parse_args()
    if len(args) != 3:
        parser.error("you must specify a stack or app name, version, and the location of the distro.yaml for this release")
    name, version, distro_file = args
    if not os.path.isfile(distro_file):
        parser.error("[%s] does not appear to be a valid file"%distro_file)
    return name, version, distro_file        

def load_and_validate_properties():
    """
    @return: name, version, distro_file, source_dir, distro, release_props
    @rtype: (str, str, str, str, str, dict)
    """
    name, version, distro_file = load_sys_args()
    # for now, we only work with stacks
    source_dir = roslib.stacks.get_stack_dir(name)
    if not source_dir:
            #TODO: check for app instead
        raise ReleaseException("cannot locate stack [%s]"%name)

    check_svn_status(source_dir)
    
    # figure out what we're releasing against
    distro = get_active_distro()
    print "Distribution target is [%s]"%distro

    release_props = load_release_props(name, distro, distro_file, source_dir)
    print_bold("Release Properties")
    for k, v in release_props.iteritems():
        print " * %s: %s"%(k, v)

    # brittle test to make sure that user got the args correct
    if not '.' in version:
        raise ReleaseException("hmm, [%s] doesn't look like a version number to me"%version)
    cmake_version = get_version(name, source_dir)
    if cmake_version != version:
        raise ReleaseException("version number in CMakeLists.txt appears to be incorrect:\n\n%s"%cmake_version)
    
    return name, version, distro_file, source_dir, distro, release_props    
        
def main():
    try:
        props = load_and_validate_properties()
        name, version, distro_file, source_dir, distro, release_props = props[:6]

        if 1:
            dist = make_dist(name, version, distro, source_dir, release_props)

        if 1:
            tag_urls = tag_subversion(name, version, distro, release_props)

        if 1:
            update_rosdistro_yaml(name, version, distro, distro_file)

    except ReleaseException, e:
        print >> sys.stderr, "ERROR: %s"%str(e)
        sys.exit(1)

def get_version(name, source_dir):
    cmake_p = os.path.join(source_dir, 'CMakeLists.txt')
    print "Reading version number from %s"%cmake_p
    with open(cmake_p) as f:
        for l in f.readlines():
            if l.strip().startswith('rosbuild_make_distribution'):
                import re
                x_re = re.compile(r'[()]')
                lsplit = x_re.split(l.strip())
                if len(lsplit) < 2:
                    raise ReleaseException("couldn't find version number in CMakeLists.txt:\n\n%s"%l)
                return lsplit[1]

    
def update_rosdistro_yaml(name, version, distro, distro_file):
    if not os.path.exists(distro_file):
        raise ReleaseException("[%s] does not exist"%distro_file)

    with open(distro_file) as f:
        d = [d for d in yaml.load_all(f.read())]
        if len(d) != 1:
            raise ReleaseException("found more than one release document in [%s]"%distro_file)
        d = d[0]

    distro_d = d
    if not 'stacks' in d:
        d['stacks'] = {}
    d = d['stacks']
    if not name in d:
        d[name] = {}
    d = d[name]
    # set the version key, assume not overriding properties
    d[distro] = str(version)

    print "Writing new release properties to [%s]"%distro_file
    with open(distro_file, 'w') as f:
        f.write(yaml.safe_dump(distro_d))
    
def tag_subversion(name, version, distro, release_props):
    urls = []
    for k in ['release-svn', 'distro-svn']:
        from_url = expand_uri(release_props['dev-svn'], name, version, distro, \
                                  release_props['os_name'], release_props['os_ver'])
        tag_url = expand_uri(release_props[k], name, version, distro, \
                                 release_props['os_name'], release_props['os_ver'])
        
        release_name = "%s-%s"%(name, version)

        cmds = []
        # delete old svn tag if it's present
        append_rm_if_exists(tag_url, cmds, 'Deleting old tag')
        # svn cp command to create new tag
        cmds.append(['svn', 'cp', '--parents', '-m', 'Tagging %s release'%release_name, from_url, tag_url])
        if not ask_and_call(cmds):    
            print "create_release will not create this tag in subversion"
        else:
            urls.append(tag_url)
    return urls
        
def get_active_distro():
    from subprocess import Popen, PIPE
    try:
        #TODO: this logic won't work
        output = Popen(['rosdistro', 'active'], stdout=PIPE, stderr=PIPE).communicate()
    except:
        output = [None]

    if not output[0]:
        return 'latest'
    else:
        return output[0].strip()
    
def load_release_props(name, distro, distro_file, stack_dir):
    print "Loading uri rules from %s"%distro_file
    if not os.path.isfile(distro_file):
        raise ReleaseException("Cannot find [%s].\nPlease consult documentation on how to create this file"%p)
    with open(distro_file) as f:
        docs = [d for d in yaml.load_all(f.read())]
        if len(docs) != 1:
            raise ReleaseException("Found multiple YAML documents in [%s]"%distro_file)
        doc = docs[0]

    # there are three tiers of dictionaries that we look in for uri rules
    rules_d = [doc.get('stacks', {}),
               doc.get('stacks', {}).get(name, {}),
               doc.get('stacks', {}).get(name, {}).get(distro, {})]
    rules_d = [d for d in rules_d if d]
    # load the '_uri_rules' from the dictionaries, in order
    props = {}
    for d in rules_d:
        if type(d) == dict:
            props.update(d.get('_uri_rules', {}))

    if not props:
        raise ReleaseException("[%s] is missing _uri_rules. Please consult documentation"%(distro_file))
    
    for reqd in ['release-svn']:
        if not reqd in props:
            raise ReleaseException("[%s] is missing required key [%s]"%(distro_file, reqd))

    # add in some additional keys
    if not 'dev-svn' in props:
        from subprocess import Popen, PIPE
        output = Popen(['svn', 'info'], stdout=PIPE, cwd=stack_dir).communicate()[0]
        url_line = [l for l in output.split('\n') if l.startswith('URL:')]
        if url_line:
            props['dev-svn'] = url_line[0][4:].strip()
        else:
            raise ReleaseException("cannot determine SVN URL of stack [%s]"%name)
    
    os_name, os_ver = detect_os()
    props['os_name'] = os_name
    props['os_ver'] = os_ver

    return props
           
    
def print_bold(m):
    print '\033[1m%s\033[0m'%m    

def checkout_svn(name, uri):
    import tempfile
    tmp_dir = tempfile.mktemp()
    dest = os.path.join(tmp_dir, name)
    print 'Checking out a fresh copy of %s from %s to %s...'%(name, uri, dest)
    cmd = ['svn', 'co', uri, dest]
    check_call(cmd)
    return tmp_dir

def make_dist(name, version, distro, source_dir, release_props):
    from_url = expand_uri(release_props['dev-svn'], name, version, distro, \
                             release_props['os_name'], release_props['os_ver'])
    
    tmp_dir = checkout_svn(name, from_url)
    tmp_source_dir = os.path.join(tmp_dir, name)
    print 'Building a distribution for %s in %s'%(name, tmp_source_dir)
    cmd = ['make', 'package_source']
    check_call(cmd, cwd=tmp_source_dir)
    tarball = "%s-%s.tar.bz2"%(name, version)
    import shutil
    src = os.path.join(tmp_source_dir, 'build', tarball)
    dst = os.path.join(source_dir, tarball)
    shutil.copyfile(src, dst)
    shutil.rmtree(tmp_dir)
    print_bold("Release should be in %s"%dst)
    return dst
                
# this is copied from rosdistro -- need to change how
# rosdistro works so that it can be used as a library    
def expand_uri(rule, stack_name, stack_ver, distro_name, os_name, os_ver):
  s = rule.replace('$STACK_NAME', stack_name)
  s =    s.replace('$STACK_VERSION', stack_ver)
  s =    s.replace('$DISTRO_NAME', distro_name)
  s =    s.replace('$OS_NAME', os_name)
  s =    s.replace('$OS_VERSION', os_ver)
  return s


# this is copied from rosdistro -- need to change how
# rosdistro works so that it can be used as a library    
def detect_os():
  os_ver = ''
  os_name = os.environ.get('ROS_OS_NAME')
  if os_name is not None:
    os_ver = os.environ.get('ROS_OS_VERSION')
    if os_ver is None:
      os_ver = ''
    return os_name, os_ver
  if os.path.isfile('/etc/arch-release'):
    os_name = 'arch'
  elif os.path.isfile('/etc/redhat-release'):
    f = open('/etc/redhat_release', 'r')
    tokens = f.readline().split()
    if len(tokens) < 3 or tokens[0] != 'Fedora' or tokens[1] != 'release':
      print >> sys.stderr, "unknown /etc/redhat-release file found."
      exit(1)
    os_name = 'fedora'
    os_ver = tokens[2]
  elif os.path.isfile('/etc/gentoo-release'):
    f = open('/etc/gentoo-release', 'r')
    tokens = f.readline().split()
    if len(tokens) < 2 or tokens[0] != 'Gentoo':
      print >> sys.stderr, "unknown /etc/gentoo-release file found."
      exit(1)
    os_name = 'gentoo'
  elif os.path.isfile('/etc/issue'):
    f = open('/etc/issue', 'r')
    tokens = f.readline().split()
    if len(tokens) < 2 or (tokens[0] != 'Ubuntu' and tokens[0] != 'Debian'):
      print >> sys.stderr, "unknown /etc/issue file found."
      exit(1)
    if tokens[0] == 'Ubuntu':
      os_name = 'ubuntu'
      version_tokens = tokens[1].split('.')
      if len(version_tokens) < 2:
        print >> sys.stderr, "unknown version format in /etc/issue"
        exit(1)
      os_ver = version_tokens[0] + '.' + version_tokens[1]
    elif tokens[1] == 'Debian':
      os_name = 'debian'
      if tokens[2] == '5.0':
        os_ver = 'lenny'
      elif tokens[2] == '4.0':
        os_ver = 'etch'
      elif tokens[2] == 'squeeze/sid':
        os_ver = 'squeeze/sid'
      else:
        print >> sys.stderr, "unknown debian version in /etc/issue"
        exit(1)
    else:
      print >> sys.stderr, "unknown /etc/issue file"
      exit(1)
  elif os.path.isfile('/usr/bin/sw_vers'):
    os_name = 'macports' # assume this is the only decent way to get things
    p = Popen(['sw_vers','-productVersion'], stdout = PIPE, stderr = PIPE)
    sw_vers_stdout, sw_vers_stderr = p.communicate()
    ver_tokens = sw_vers_stdout.strip().split('.')
    os_ver = ver_tokens[0] + '.' + ver_tokens[1]
  return os_name, os_ver

def check_svn_status(source_dir):
    """make sure that all outstanding code has been checked in"""
    cmd = ['svn', 'st', '-q']
    output = Popen(cmd, stdout=PIPE, stderr=PIPE, cwd=source_dir).communicate()
    if output[0]:
        raise ReleaseException("svn status in stack reported uncommitted files:\n%s"%output[0])
    if output[1]:
        raise ReleaseException("svn status in [%s] reported errors:\n%s"%(source_dir, output[0]))

def append_rm_if_exists(url, cmds, msg):
    cmd = ['svn', 'ls', url]
    output = Popen(cmd, stdout=PIPE, stderr=PIPE).communicate()
    if output[0]:
        cmds.append(['svn', 'rm', '-m', msg, url]) 

## Pretty print cmds, ask if they should be run, and if so, runs them.
## Returns true if they were run.
def ask_and_call(cmds):
    # Pretty-print a string version of the commands
    def quote(s):
        return '"%s"'%s if ' ' in s else s
    print "Okay to execute:\n\n%s\n(y/n)?"%('\n'.join([' '.join([quote(s) for s in c]) for c in cmds]))
    while 1:
        input = sys.stdin.readline().strip()
        if input in ['y', 'n']:
            break
    accepted = input == 'y'
    if accepted:
        for c in cmds:
            check_call(c)
    return accepted
    
if __name__ == '__main__':
    main()
