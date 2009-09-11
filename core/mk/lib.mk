CCC=g++
TMP = $(SRC:.cpp=.o)
OBJ = $(TMP:.cc=.o)
AUTOGEN_INCLUDES = $(shell rospack --lang=cpp --attrib=cflags export $(PKG))
ALL_CFLAGS = $(CFLAGS) -Wall -g -O2 $(AUTOGEN_INCLUDES)

all: depend $(OUT)

$(OUT): $(OBJ)
	-mkdir -p `dirname $(OUT)`
	ar rcs $(OUT) $(OBJ)
.cpp.o:
	${CCC} -g -O2 ${ALL_CFLAGS} -c $< -o $@
.cc.o:
	${CCC} -g -O2 ${ALL_CFLAGS} -c $< -o $@

depend: 
	g++ ${ALL_CFLAGS} -MM $(SRC) >depend

clean:
	-rm depend 
	-rm $(OBJ)
	-rm $(OUT)

ifeq (,$(findstring clean,$(MAKECMDGOALS)))
ifeq (,$(findstring wipe,$(MAKECMDGOALS)))
-include depend
endif
endif
.PHONY : clean
