LIB := $(shell faust --libdir)
INC := $(shell faust --includedir) 

INC_WASMTIME := ../../target/c-api/include/ 
LIB_WASMTIME := ../../target/release/libwasmtime.a 

DESTDIR ?=
PREFIX ?= /usr/local

prefix := $(DESTDIR)$(PREFIX)

all: faustwastime faustbench-wasmtime faustwastime-c faustbench-wasmtime-c

faustwastime: faustwasmtime.cpp wasmtime_dsp.cpp
	$(CXX) -std=c++17 -O3 faustwasmtime.cpp  wasmtime_dsp.cpp -I $(INC) -I $(INC_WASMTIME) $(LIB)/libHTTPDFaust.a -L$(LIB) $(LIB_WASMTIME) `pkg-config --cflags --libs jack libmicrohttpd` -o faustwastime

faustbench-wasmtime: faustbench-wasmtime.cpp wasmtime_dsp.cpp
	$(CXX) -std=c++17 -O3 faustbench-wasmtime.cpp wasmtime_dsp.cpp -I $(INC) -I $(INC_WASMTIME) -L$(LIB) $(LIB_WASMTIME) -o faustbench-wasmtime

faustwastime-c: faustwasmtime.cpp wasmtime_dsp.c
	$(CXX) -std=c++17 -O3 faustwasmtime.cpp  wasmtime_dsp.c -DWASMTIME_C -I $(INC) -I $(INC_WASMTIME) $(LIB)/libHTTPDFaust.a -L$(LIB) $(LIB_WASMTIME) `pkg-config --cflags --libs jack libmicrohttpd` -o faustwastime-c

faustbench-wasmtime-c: faustbench-wasmtime.cpp wasmtime_dsp.cpp
	$(CXX) -std=c++17 -O3 faustbench-wasmtime.cpp wasmtime_dsp.c -DWASMTIME_C -I $(INC) -I $(INC_WASMTIME) -L$(LIB) $(LIB_WASMTIME) -o faustbench-wasmtime-c

format:
	find . -iname '*.cpp' -execdir clang-format -i -style=file {} \;
	find . -iname '*.hh' -execdir clang-format -i -style=file {} \;
	find . -iname '*.c' -execdir clang-format -i -style=file {} \;
	find . -iname '*.h' -execdir clang-format -i -style=file {} \;

install: 
	([ -e faustwastime ]) && cp faustwastime $(prefix)/bin
	([ -e faustbench-wasmtime ]) && cp faustbench-wasmtime $(prefix)/bin
	([ -e faustwastime-c ]) && cp faustwastime-c $(prefix)/bin
	([ -e faustbench-wasmtime-c ]) && cp faustbench-wasmtime-c $(prefix)/bin

clean:
	rm -f faustwastime faustbench-wasmtime faustwastime-c faustbench-wasmtime-c
	