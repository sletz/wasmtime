LIB := $(shell faust --libdir)
INC := $(shell faust --includedir) 

INC_WASMTIME := ../../target/c-api/include/ 
LIB_WASMTIME := ../../target/release/libwasmtime.a 

DESTDIR ?=
PREFIX ?= /usr/local

prefix := $(DESTDIR)$(PREFIX)

all: faustwastime faustbench-wasmtime

faustwastime: faustwasmtime.cpp wasmtime_dsp.cpp
	$(CXX) -std=c++17 -O3 faustwasmtime.cpp  wasmtime_dsp.cpp -I $(INC) -I $(INC_WASMTIME) $(LIB)/libHTTPDFaust.a -L$(LIB) $(LIB_WASMTIME) `pkg-config --cflags --libs jack libmicrohttpd` -o faustwastime

faustbench-wasmtime: faustbench-wasmtime.cpp wasmtime_dsp.cpp
	$(CXX) -std=c++17 -O3 faustbench-wasmtime.cpp wasmtime_dsp.cpp -I $(INC) -I $(INC_WASMTIME) -L$(LIB) $(LIB_WASMTIME) -o faustbench-wasmtime

format:
	find . -iname '*.cpp' -execdir clang-format -i -style=file {} \;
	find . -iname '*.h' -execdir clang-format -i -style=file {} \;

install: 
	([ -e faustwastimet ]) && cpfaustwastime $(prefix)/bin
	([ -e faustbench-wasmtime ]) && cp faustbench-wasmtime $(prefix)/bin

clean:
	rm -f faustwastime faustbench-wasmtime
	