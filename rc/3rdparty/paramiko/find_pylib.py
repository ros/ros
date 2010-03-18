import os
import sys

def find_python_path(pkg, build_dir):
    # find the lib dir
    for p in os.listdir(build_dir):
        if p.startswith('lib'):
            candidate = os.path.join(build_dir, p, pkg)
            if os.path.isdir(candidate):
                return os.path.join(build_dir, p)
    
if __name__ == "__main__":
    import optparse
    parser = optparse.OptionParser(usage="usage: build_manifest.py <package> <build-dir>")
    options, args = parser.parse_args()
    if len(args) != 2:
        parser.error("You must specify a package and build lib directory only")
    pkg, build_dir = args

    if not os.path.isdir(build_dir):
        parser.error("build-dir is not a directory")
    
    python_path = find_python_path(pkg, build_dir)
    if not python_path:
        print sys.stderr, "Cannot find python library"
        sys.exit(1)
    print python_path
    
    
    
