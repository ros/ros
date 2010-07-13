# not sure this is necessary
wd  := $(realpath $(CURDIR)/..)

pkg      := $(notdir $(wd))
msgfiles := $(wildcard *.msg)
msgcpp   := $(addprefix cpp/$(pkg)/,$(msgfiles:.msg=.cpp))
msgh     := $(addprefix cpp/$(pkg)/,$(msgfiles:.msg=.h))

all: $(msgh)

%.h : ../../%.msg
	`rospack find roslib`/scripts/genmsg $@
	@echo $@ $<

cpp:
	mkdir -p cpp


test:
	echo $(pkg)
	echo $(msgcpp)
