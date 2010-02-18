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
    @return: name, version, distro_file, distro_name
    @rtype: (str, str, str, str)
    """
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog <stack> <version> <release-file>", prog=NAME)
    options, args = parser.parse_args()
    if len(args) != 3:
        parser.error("""You must specify: 
 * stack name (e.g. common_msgs)
 * version (e.g. 1.0.1)
 * release file (e.g. latest.rosdistro)""")
    name, version, distro_file = args
    if not os.path.isfile(distro_file):
        parser.error("[%s] does not appear to be a valid file"%distro_file)
    if not distro_file.endswith('.rosdistro'):
        parser.error("rosdistro file [%s] must end with a .rosdistro extension"%distro_file)
    distro_name = distro_file[:-len('.rosdistro')]
    return name, version, distro_file, distro_name

def load_and_validate_properties():
    """
    @return: name, version, distro_file, source_dir, distro, release_props
    @rtype: (str, str, str, str, str, dict)
    """
    name, version, distro_file = load_sys_args()
    # for now, we only work with stacks
    try:
        source_dir = roslib.stacks.get_stack_dir(name)
    except:
        raise ReleaseException("cannot locate stack [%s]"%name)

    check_svn_status(source_dir)
    
    release_props = load_release_props(name, distro_file, source_dir)
    print_bold("Release Properties")
    for k, v in release_props.iteritems():
        print " * %s: %s"%(k, v)
    release = release_props['release']
    # figure out what we're releasing against
    print "Release target is [%s]"%release


    # brittle test to make sure that user got the args correct
    if not '.' in version:
        raise ReleaseException("hmm, [%s] doesn't look like a version number to me"%version)
    cmake_version = get_version(name, source_dir)
    if cmake_version != version:
        raise ReleaseException("version number in CMakeLists.txt appears to be incorrect:\n\n%s"%cmake_version)
    
    return name, version, distro_file, source_dir, release_props    
        
def main():
    try:
        props = load_and_validate_properties()
        name, version, distro_file, source_dir, release_props = props[:6]

        if 1:
            dist = make_dist(name, version, source_dir, release_props)

        if 1:
            tag_urls = tag_subversion(name, version, release_props)

        if 1:
            update_rosdistro_yaml(name, version, distro_file)

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

    
def update_rosdistro_yaml(name, version, distro_file):
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
    d['version'] = str(version)

    print "Writing new release properties to [%s]"%distro_file
    with open(distro_file, 'w') as f:
        f.write(yaml.safe_dump(distro_d))

def tag_subversion(name, version, release_props):
    urls = []
    release = release_props['release']
    for k in ['release-svn', 'distro-svn']:
        from_url = expand_uri(release_props['dev-svn'], name, version, release)
        tag_url = expand_uri(release_props[k], name, version, release)
        
        release_name = "%s-%s"%(name, version)

        cmds = []
        # delete old svn tag if it's present
        append_rm_if_exists(tag_url, cmds, 'Making room for new release')
        # svn cp command to create new tag
        cmds.append(['svn', 'cp', '--parents', '-m', 'Tagging %s new release'%release_name, from_url, tag_url])
        if not ask_and_call(cmds):    
            print "create_release will not create this tag in subversion"
        else:
            urls.append(tag_url)
    return urls
        
def load_release_props(name, distro_file, stack_dir):
    print "Loading uri rules from %s"%distro_file
    if not os.path.isfile(distro_file):
        raise ReleaseException("Cannot find [%s].\nPlease consult documentation on how to create this file"%p)
    with open(distro_file) as f:
        docs = [d for d in yaml.load_all(f.read())]
        if len(docs) != 1:
            raise ReleaseException("Found multiple YAML documents in [%s]"%distro_file)
        doc = docs[0]

    # there are two tiers of dictionaries that we look in for uri rules
    rules_d = [doc.get('stacks', {}),
               doc.get('stacks', {}).get(name, {})]
    rules_d = [d for d in rules_d if d]
    # load the '_rules' from the dictionaries, in order
    props = {}
    for d in rules_d:
        if type(d) == dict:
            props.update(d.get('_rules', {}))

    if not props:
        raise ReleaseException("[%s] is missing '_rules'. Please consult documentation"%(distro_file))
    
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

def make_dist(name, version, source_dir, release_props):
    from_url = expand_uri(release_props['dev-svn'], name, version, release_props['release'])
    
    tmp_dir = checkout_svn(name, from_url)
    tmp_source_dir = os.path.join(tmp_dir, name)
    print 'Building a distribution for %s in %s'%(name, tmp_source_dir)
    cmd = ['make', 'package_source']
    try:
        check_call(cmd, cwd=tmp_source_dir)
    except:
        raise ReleaseException("unable to 'make package_source' in package. Most likely the Makefile and CMakeLists.txt files have not been checked in")
    tarball = "%s-%s.tar.bz2"%(name, version)
    import shutil
    src = os.path.join(tmp_source_dir, 'build', tarball)
    dst = os.path.join(source_dir, tarball)
    shutil.copyfile(src, dst)
    shutil.rmtree(tmp_dir)
    print_bold("Release should be in %s"%dst)
    return dst

def expand_uri(rule, stack_name, stack_ver, release_name):
  s = rule.replace('$STACK_NAME', stack_name)
  s =    s.replace('$STACK_VERSION', stack_ver)
  s =    s.replace('$RELEASE_NAME', release_name)
  return s

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
