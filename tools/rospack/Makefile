# Can't include cmake.mk because we need rospack to find it, and we're
# trying to build rospack.  So we inline a stripped-down version of
# cmake.mk.
#include $(shell rospack find mk)/cmake.mk

# The all target does the heavy lifting, creating the build directory and
# invoking CMake
all:
	@mkdir -p build
	-mkdir -p bin
	@if ! (cd build && cmake ..); then \
	  echo "[rosbuild] CMake failed; trying to clean and start over"; \
	  make clean; \
	  mkdir -p build; \
	  cd build && cmake ..; \
	fi
	cd build && make $(PARALLEL_JOBS)

install: all
	cd build && make install

# The clean target blows everything away
clean:
	-cd build && make clean
	rm -rf build

test: all
	if cd build && make -k $@; then make test-results; else make test-results && exit 1; fi
tests: all
	cd build && make $@
test-future: all
	cd build && make -k $@
gcoverage: all
	cd build && make $@

#SRC = main.cpp
#LIBSRC = rospack.cpp \
#      tinyxml-2.5.3/tinystr.cpp \
#			tinyxml-2.5.3/tinyxml.cpp \
#			tinyxml-2.5.3/tinyxmlparser.cpp \
#			tinyxml-2.5.3/tinyxmlerror.cpp
#
#
#OBJ = $(SRC:.cpp=.o)
#LIBOBJ = $(LIBSRC:.cpp=.o)
#OUT = ../../bin/rospack
#LIBOUT = librospack.a
#
#CC=g++
#AR=ar
#LIBS=-lm
#CFLAGS= -O3 -Wall -fPIC
## For code coverage
##LIBS=-lm -lgcov
##CFLAGS= -O0 -Wall -g -fprofile-arcs -ftest-coverage
#INCLUDES=-I.
#
#default: depend $(OUT) $(LIBOUT)
#
#$(OUT): $(OBJ) $(LIBOUT)
#	${CC} $(OBJ) -o $(OUT) $(LIBS) -L. -lrospack
#
#$(LIBOUT): $(LIBOBJ)
#	${AR} cr $@ $(LIBOBJ)
#
#.cpp.o:
#	${CC} ${CFLAGS} ${INCLUDES} -c $< -o $@
#
#depend: $(SRC) $(LIBSRC)
#	gcc $(INCLUDES) -MM $(SRC) $(LIBSRC) >depend
#			  
#clean:
#	rm -f depend $(OBJ) $(OUT) $(LIBOUT) $(LIBOBJ)
#
#-include depend
