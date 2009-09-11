# A target to test building of a package, all of its dependencies, and all
# of the things that depend on it

# HACK: assume that the package name is the name of the directory we're
# sitting in
pkg = $(shell basename $(PWD))

rosalldeps = $(shell rospack find rospack)/rosalldeps

deps = $(shell $(rosalldeps) $(pkg))

rules = $(foreach d, $(deps), cd $(shell rospack find $(d)) && if test -f Makefile -a ! -f ROS_NOBUILD -a ! -f ROS_BUILD_BLACKLIST ; then make; else echo "skipping $(d)"; fi &&) true;

buildtest:
	$(rules)
