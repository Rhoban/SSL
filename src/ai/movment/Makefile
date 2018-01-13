CPPFLAGS=-I/usr/include/eigen3
LDFLAGS=-lstdc++ -lm
CXXFLAGS=-Wall -O0 -g
 
all: test_movment 

clean:
	-rm -rf *.o
	-rm -rf test_movment 

test_movment: test_movment.o movment.o

movment.o: movment.cpp movment.h

test_movment.o: test_movment.cpp

.phony: all clean
