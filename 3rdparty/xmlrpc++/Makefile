include $(shell rospack find mk)/cmake.mk
# # makefile written for gnu make
# CXX		= g++
# SRC		= ./src
# CPPFLAGS	= -I$(SRC)
# DEBUG		= -g
# OPTIMIZE	= -O2
# GCCWARN		= -Wall
# CXXFLAGS	= $(DEBUG) $(GCCWARN) $(OPTIMIZE) $(INCLUDES)
# 
# LIB		= ./libXmlRpc.a
# SOLIB = libXmlRpc.so.1
# 
# # Add your system-dependent network libs here. These are
# # only used to build the tests (your application will need them too).
# # Linux: none
# # Solaris: -lsocket -lnsl
# #SYSTEMLIBS	= -lsocket -lnsl
# SYSTEMLIBS	=
# LDLIBS		= $(LIB) $(SYSTEMLIBS)
# 
# OBJ		= $(SRC)/XmlRpcClient.o $(SRC)/XmlRpcDispatch.o \
# 		$(SRC)/XmlRpcServer.o $(SRC)/XmlRpcServerConnection.o \
# 		$(SRC)/XmlRpcServerMethod.o $(SRC)/XmlRpcSocket.o $(SRC)/XmlRpcSource.o \
# 		$(SRC)/XmlRpcUtil.o $(SRC)/XmlRpcValue.o
# 
# #all:		$(LIB) tests
# # The SOLIB target does not work on OS X. 
# #all:		$(LIB) $(SOLIB)
# all:		$(LIB)
# 
# $(LIB):		$(OBJ)
# 		$(AR) $(ARFLAGS) $(LIB) $(OBJ)
# $(SOLIB): $(OBJ)
# 		gcc -shared -Wl,-soname,$(SOLIB) -o $(SOLIB) $(OBJ)
# 
# 
# tests:		$(LIB)
# 		cd test && $(MAKE) CXX=$(CXX) CXXFLAGS="$(CXXFLAGS)" SYSTEMLIBS="$(SYSTEMLIBS)"
# 
# doc doxygen:
# 		cd src && doxygen Doxyfile
# 
# clean:
# 		rm -f $(SRC)/*.o
# 		rm -f $(SRC)/*~
# 		rm -f $(LIB)
# 		cd test && $(MAKE) clean
# 
