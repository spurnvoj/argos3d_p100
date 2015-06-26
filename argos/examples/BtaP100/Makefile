CPP = $(CROSS_COMPILE)g++
CFLAGS += -Wall -fpic -I../../inc -O2

LIBPATH = ../../lib/BtaP100/x64

all: bta_p100_example_minimal bta_p100_example

bta_p100_example_minimal: bta_p100_example_minimal.cpp
	$(CPP) $(CFLAGS) -c bta_p100_example_minimal.cpp
	$(CPP) -o bta_p100_example_minimal bta_p100_example_minimal.o -L$(LIBPATH) -Wl,-rpath=$(LIBPATH) -lbta_p100 -lrt

bta_p100_example: bta_p100_example.cpp
	$(CPP) $(CFLAGS) -c bta_p100_example.cpp
	$(CPP) -o bta_p100_example bta_p100_example.o -L$(LIBPATH) -Wl,-rpath=$(LIBPATH) -lbta_p100 -lrt
