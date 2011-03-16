# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
# Revision $Id: genmsg_py.py 1590 2008-07-22 18:30:01Z sfkwc $

## Common generation tools for Python ROS message and service generators

import os
import errno  # for smart handling of exceptions for os.makedirs()
import sys
import traceback
import glob

import rosidl.msgs
import rosidl.packages
from rosidl.genpy import MsgGenerationException

class Generator(object):
    
    ## @param name str: name of resource types
    ## @param ext str: file extension of resources (e.g. '.msg')
    ## @param subdir str: directory sub-path of resources (e.g. 'msg')
    ## @param exception class: exception class to raise if an error occurs
    def __init__(self, name, what, ext, subdir, exception):
        self.name = name
        self.what = what
        self.subdir = subdir
        self.ext = ext
        self.exception = exception
    
    ## @return [str]: list of (message/service) types in specified package
    def list_types(self, package):
        ext = self.ext
        types = rosidl.resources.list_package_resources(
            package, False, self.subdir,
            lambda x: os.path.isfile(x) and x.endswith(ext))
        return [x[:-len(ext)] for x in types]

    ## Convert resource filename to ROS resource name
    ## @param filename str: path to .msg/.srv file
    ## @return str: name of ROS resource
    def resource_name(self, filename):
        assert isinstance(filename, str)
        return os.path.basename(filename)[:-len(self.ext)]
        
    ## @param type_name str: Name of message type sans package,
    ## e.g. 'String'
    ## @return str: name of python module for auto-generated code
    def _module_name(self, type_name):
        return "_"+type_name
    
    ## @param outdir str: path to directory that files are generated to
    ## @return str: output file path based on input file name and output directory
    def outfile_name(self, outdir, infile_name):
        assert isinstance(outdir, str)
        assert isinstance(infile_name, str)
        # Use leading _ so that module name does not collide with message name. It also
        # makes it more clear that the .py file should not be imported directly
        return os.path.join(outdir, self._module_name(self.resource_name(infile_name))+".py")

    ## @param base_dir str: base directory of package
    ## @return str: out directory for generated files. For messages, this is
    ## package 'mypackage.msg', stored in the src directory (.srv for services).
    # - this is different from C++ and Ruby, mainly because Python cannot unify packages
    #   across multiple directories. The msg subpackage is because we want to cleanly
    #   denote auto-generated files.
    def outdir(self, basedir):
        "compute the directory that the .py files are output to"
        outdir = os.path.join(basedir, rosidl.packages.SRC_DIR, os.path.basename(basedir), self.subdir)
        if not os.path.exists(outdir):
            try:
                os.makedirs(outdir)
            except Exception, e:
                # It's not a problem if the directory already exists,
                # because this can happen during a parallel build
                if e.errno != errno.EEXIST:
                    raise e
                
        elif not os.path.isdir(outdir): 
            raise self.exception("Cannot write to %s: file in the way"%outdir)
        return outdir

    def generate(self, package, f, outdir, incdir):
        raise Exception('subclass must override')

    ## reindex files as a dictionary keyed by package
    def generate_package_files(self, package_files, files, ext):
        files = filter(lambda f: f.endswith(ext), files)
        retcode = 0
        assert isinstance(files, list)
        for f in files:

            try:
                package_dir, package = rosidl.packages.get_dir_pkg(f)
                outdir = self.outdir(package_dir)
                if not package:
                    raise self.exception("Cannot locate package for %s. Is ROS_ROOT set?"%f)
                outfile_name = self.outfile_name(outdir, f)
                if not package in package_files:
                    package_files[package] = []
                package_files[package].append(f)
            except Exception, e:
                print "\nERROR[%s]: Unable to load %s file '%s': %s\n"%(self.name, self.ext, f, e)
                raise
        return retcode

    def write_modules(self, package_files, options):
        for package, pfiles in package_files.iteritems():
            mfiles = map(lambda s: os.path.basename(os.path.splitext(s)[0]),
                         pfiles)
            outdir = options.outdir

            #TODO: also check against MSG/SRV dir to make sure it
            # really is a generated file get a list of all the python
            # files in the generated directory so we can import them
            # into __init__.py. We intersect that list with the list
            # of the .msg files so we can catch deletions without
            # having to 'make clean'
            good_types = set([f[1:-3] for f in os.listdir(outdir)
                             if f.endswith('.py') and f != '__init__.py'])
            types = set(map(lambda s: os.path.basename(os.path.splitext(s)[0]),
                            pfiles))
            generated_modules = [self._module_name(f) for f in good_types.intersection(types)]

            self.write_module(options.outdir, package, generated_modules, options.srcdir)
        return 0

    ## @param base_dir str: path to package
    ## @param package str: name of package to write module for
    ## @param generated_modules [str]: list of generated message modules,
    ##   i.e. the names of the .py files that were generated for each
    ##   .msg file.
    def write_module(self, basedir, package, generated_modules, srcdir):
        """create a module file to mark directory for python"""
        if not os.path.exists(basedir):
            os.makedirs(basedir)
        elif not os.path.isdir(basedir):
            raise self.exception("file preventing the creating of module directory: %s"%dir)
        p = os.path.join(basedir, '__init__.py')
        f = open(p, 'w')
        try:
            #this causes more problems than anticipated -- for pure python
            #packages it works fine, but in C++ packages doxygen seems to prefer python first.
            #f.write('## \mainpage\n') #doxygen
            #f.write('# \htmlinclude manifest.html\n')
            for mod in generated_modules:
                f.write('from %s import *\n'%mod)
        finally:
            f.close()

        parentInit = os.path.dirname(basedir)
        p = os.path.join(parentInit, '__init__.py')
        if not os.path.exists(p):
            #touch __init__.py in the parent package
            f = open(p, 'w')
            print >>f, "import pkgutil, os.path"
            print >>f, "__path__ = pkgutil.extend_path(__path__, __name__)"
            staticinit = '%s/%s/__init__.py' % (srcdir, package)
            print >>f, "if os.path.isfile('%s'): execfile('%s')" % (staticinit, staticinit)
            f.close()

    def generate_package(self, package, pfiles, options):
        if not rosidl.names.is_legal_resource_base_name(package):
            print "\nERROR[%s]: package name '%s' is illegal and cannot be used in message generation.\nPlease see http://ros.org/wiki/Names"%(self.name, package)
            return 1 # flag error
        
        #package_dir = rosidl.packages.get_pkg_dir(package, True)
        #if package_dir is None:
        #print "\nERROR[%s]: Unable to locate package '%s'\n"%(self.name, package)
        #return 1 #flag error

        # package/src/package/msg for messages, packages/src/package/srv for services
        outdir = options.outdir #self.outdir(package_dir)
        try:
            # TODO: this can result in packages getting dependencies
            # that they shouldn't. To implement this correctly will
            # require an overhaul of the message generator
            # infrastructure.
            #rosidl.msgs.load_package_dependencies(package, load_recursive=True)
            pass
        except Exception, e:
            #traceback.print_exc()
            print "\nERROR[%s]: Unable to load package dependencies for %s: %s\n"%(self.name, package, e)
            return 1 #flag error
        try:
            # rosidl.msgs.load_package(package)        
            pass
        except Exception, e:
            print "\nERROR[%s]: Unable to load package %s: %s\n"%(self.name, package, e)
            return 1 #flag error

        retcode = 0
        for f in pfiles:
            try:
                outfile = self.generate(package, f, outdir, options.includepath) #actual generation
            except Exception, e:
                if not isinstance(e, MsgGenerationException) and not isinstance(e, rosidl.msgs.MsgSpecException):
                    traceback.print_exc()
                print "\nERROR[%s]: Unable to generate %s for package '%s': while processing '%s': %s\n"%(self.name, self.what, package, f, e)
                retcode = 1 #flag error
        return retcode
        
    def generate_all_by_package(self, package_files, options):
        """
        @return: return code
        @rtype: int
        """
        retcode = 0
        for package, pfiles in package_files.iteritems():
            retcode = self.generate_package(package, pfiles, options) or retcode
        return retcode

    def generate_initpy(self, files, options):
        """
        Generate __init__.py file for each package in in the msg/srv file list
        and have the __init__.py file import the required symbols from
        the generated version of the files.
        @param files: list of msg/srv files
        @type  files: [str]
        @return: return code
        @rtype: int
        """
        
        package_files = { options.package : files }

        # pass 2: write the __init__.py file for the module
        self.write_modules(package_files, options)
        
    def generate_messages(self, files, options):
        """
        @param no_gen_initpy: if True, don't generate the __init__.py
        file. This option is for backwards API compatibility.
        @type  no_gen_initpy: bool
        @return: return code
        @rtype: int
        """
        package_files = { options.package : [files[0]] }
        # pass 1: collect list of files for each package
        # retcode = self.generate_package_files(package_files, files, self.ext)

        # pass 2: rosidl.msgs.load_package(), generate messages
        retcode = self.generate_all_by_package(package_files, options)

        # backwards compat
        if options.initpy:
            retcode = retcode or self.write_modules(package_files)
            
        return retcode

    def write_gen(self, outfile, gen, verbose):
        f = open(outfile, 'w')
        try:
            for l in gen:
                f.write(l+'\n')
        finally:
            f.close()

def usage(progname):
    print "%(progname)s file(s)"%vars()

def genmain(argv, gen, usage_fn=usage):

    from optparse import OptionParser
    parser = OptionParser("options")
    parser.add_option('--initpy', dest='initpy', action='store_true',
                      default=False)
    parser.add_option('-p', dest='package')
    parser.add_option('-s', dest='srcdir')
    parser.add_option('-o', dest='outdir')
    parser.add_option('-I', dest='includepath', action='append')
    (options, args) = parser.parse_args(argv)
    try:
        if options.initpy:
            retcode = gen.generate_initpy(args, options)
        else:
            retcode = gen.generate_messages(args[1:], options)
    except rosidl.msgs.MsgSpecException, e:
        print >> sys.stderr, "ERROR: ", e
        retcode = 1
    except MsgGenerationException, e:
        print sys.stderr, "ERROR: ",e
        retcode = 2
    except Exception, e:
        traceback.print_exc()
        print "ERROR: ",e
        retcode = 3
    sys.exit(retcode or 0)
