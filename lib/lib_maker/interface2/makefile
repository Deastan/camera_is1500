#
# Makefile for Linux
#
#-DCMAKE_CXX_FLAGS=-D__GLIBCXX_USE_CXX11_ABI=0
#CXXFLAGS=-g -stdlib=libstdc++
__GLIBCXX_USE_CXX11_ABI=0

ifeq ($(arch), 32)
    BITS = -m32
else ifeq ($(arch), 64)
    BITS = -m64
else
    # Help message
help:
	@echo "Usage: make arch=<32|64>"
endif

# Optimized or debug
OPT =       -O2
#OPT =      -ggdb

CFLAGS =	$(BITS) -fPIC -c -Wall $(OPT) -DUNIX

LDFLAGS =   $(BITS) -v
#LDFLAGS = -L/usr/lib64 -lstdc++

# Executable name
NL =		libinterface.so

INCDIRS =	-I../include
LIBS =	-L../../ -lsfaccess
CC =		g++-4.8# => WORKS g++ <= #g++-4.8#gcc-4.8 #g++
LD =		g++-4.8# => WORKS g++ <= #g++-4.8 #$(CC)#gcc-4.8 #$(CC)

SRC =		interface.cpp

OBJ =		interface.o #$(SRC:.cpp=.o)

all:		$(NL)

$(NL):		$(OBJ)
		$(LD) -shared -fPIC $(LDFLAGS) $(OBJ) $(LIBS) -o $(NL)

interface.o:	interface.cpp ../include/*.h
		$(CC) $(CFLAGS) $(INCDIRS) $<


clean:
		rm -f *.o
		rm -f $(NL)
