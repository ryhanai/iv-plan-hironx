KLTDIR	= $(HOME)/prog/jvl/extlib/klt1.3.4/klt
LIBS	+= `rospack libs-only-l opencv2`
# LIBS	+= `pkg-config --libs opencv-2.3.1`
LIBS	+= -L$(KLTDIR) -Xlinker -R$(KLTDIR) -lklt
LIBS	+= -L/usr/local/lib -lRandom
# LIBS	+= -L$(KLTDIR) -Xlinker -R$(KLTDIR) -lklt
INCS	+= `pkg-config --cflags opencv-2.3.1`
INCS	+= -I$(KLTDIR)
INCS	+= -I/usr/local/include

CP	= g++ -fPIC
CP      = g++ -fPIC
CFLAGS  = -Wall -O3 -falign-functions=4 $(INCS)
ifneq ($(ARCHDIR), Linux64)
CFLAGS  += -m32
endif

TARGET	= save load
OBJ = save.o
OBJl = load.o

all: $(TARGET)
.cpp.o:
	@$(CP) $(CFLAGS) -g -o $@ -c $< -D$(USER)



save: $(OBJ)
	@$(CP) -O3 -o $@ $^ $(LIBS) $(INCS)

load: $(OBJl)
	@$(CP) -O3 -o $@ $^ $(LIBS) $(INCS)


clean:
	@rm -f *~ *.o */*~ */*.o $(TARGET)

