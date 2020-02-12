# A target to test building of a package, all of its dependencies, and all
# of the things that depend on it
#
# For internal use only.

# HACK: assume that the package name is the name of the directory we're
# sitting in
pkg = $(shell basename $(PWD))

rosalldeps = $(shell rospack find rospack)/rosalldeps

deps = $(shell $(rosalldeps) -H 1 $(pkg))
rules = $(foreach d, $(deps), cd $(shell rospack find $(d)) && make &&) true;
.PHONY: build
build:
	$(rules)
rules_test = $(foreach d, $(deps), cd $(shell rospack find $(d)) && make test &&) true;
.PHONY: build-test
build-test:
	$(rules_test)

deps_all = $(shell $(rosalldeps) $(pkg))
rules_all = $(foreach d, $(deps_all), cd $(shell rospack find $(d)) && make &&) true;
.PHONY: build-all
build-all:
	$(rules_all)
rules_all_test = $(foreach d, $(deps_all), cd $(shell rospack find $(d)) && make test &&) true;
.PHONY: build-test-all
build-test-all:
	$(rules_all_test)
