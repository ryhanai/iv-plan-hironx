# -*- Makefile -*-

LIBIK_HIRO = ../../lib/libik_hiro.so

all: $(LIBIK_HIRO)

ik_pa10: ik_pa10.cpp
	g++ -o $@ $< -O2

ik_hiro: hironx_ikfast42.Transform6D.3_4_5_6_7_8.cpp
	g++ -lstdc++ -llapack -o $@ $< -O2

$(LIBIK_HIRO): hironx_ikfast42.Transform6D.3_4_5_6_7_8.cpp
	g++ -fPIC -I/usr/include/python2.7 \
	-DIKFAST_NO_MAIN -shared -Wl,-soname,$@ -o $@ $< -llapack -lstdc++ -lboost_python


fk_hiro: fk_hiro.cpp hironx_ikfast42.Transform6D.3_4_5_6_7_8.cpp
	g++ -llapack -lstdc++ -DIKFAST_NO_MAIN -o $@ $^

clean:
	rm -f $(LIBIK_HIRO) *.o

