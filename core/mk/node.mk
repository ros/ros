TMP = $(SRC:.cpp=.o)
OBJ = $(TMP:.cc=.o)

ifeq ($(NOAUTODEPS),)
AUTOGEN_INCLUDES := $(shell rospack --lang=cpp --attrib=cflags export $(PKG))
AUTOGEN_LIBS := $(shell rospack --lang=cpp --attrib=lflags export $(PKG))
endif

ALL_CFLAGS = $(CFLAGS) -Wall -g -O2 $(AUTOGEN_INCLUDES)
ALL_LFLAGS = $(LFLAGS) -g $(AUTOGEN_LIBS) -lm 

all: depend $(OUT)

.cpp.o:
	g++ $(ALL_CFLAGS) -c $< -o $@
.cc.o:
	g++ $(ALL_CFLAGS) -c $< -o $@

clean:
	-rm -f $(OBJ) $(OUT)
	-rm -rf depend
	-if [ -d msg ]; then rm -rf msg/cpp; fi

depend: 
ifneq ($(REQUIRES),)
	rosprereq $(REQUIRES)
endif
	@if [ -d msg ]; then $(shell rospack find roscpp)/scripts/genmsg_cpp msg/*.msg; fi
	g++ $(ALL_CFLAGS) -MM $(SRC) >depend

$(OUT): $(OBJ) $(DEPS) $(LIBDEPS)
	-mkdir `dirname $(OUT)`
	g++ $(OBJ) -o $(OUT) $(ALL_LFLAGS)

ifeq (,$(findstring clean,$(MAKECMDGOALS)))
ifeq (,$(findstring wipe,$(MAKECMDGOALS)))
-include depend
endif
endif

.PHONY : clean
