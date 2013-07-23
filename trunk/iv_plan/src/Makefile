# -*- Makefile -*-

CPQP_SRC = cpqp.cpp
CPQP_LIB = libcpqp.so
CPQP_OBJ = $(patsubst %.cpp,%.o, $(CPQP_SRC))
PQP_DIR = ../externals/pqp-1.3/PQP_v1.3
LIB_DIR = ../lib

CFLAGS = -g -O3 -Wall

all: $(LIB_DIR)/$(CPQP_LIB)

$(LIB_DIR)/$(CPQP_LIB): $(CPQP_OBJ)
	g++ -shared -g -o $@ $^ -lboost_python -lPQP -L$(PQP_DIR)/lib

$(CPQP_OBJ): $(CPQP_SRC)
	g++ $(CFLAGS) -c $< -fPIC -I/usr/include/python2.7 -I$(PQP_DIR)/include

clean: clean_objs clean_pyim
	rm -f *~ *.log

clean_objs:
	rm -rf $(LIB_DIR)/$(CPQP_LIB) $(CPQP_OBJ)

clean_pyim:
	rm -rf *.pyc parser.out parsetab.py
