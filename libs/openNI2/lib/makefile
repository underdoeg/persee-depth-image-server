CC = g++

INCPATH = -I../../ThirdParty/OpenNI2/Include \
        	        -I../../ThirdParty/LibUSB/Include \
				-I../../ThirdParty/GL/Include
				

LIBPATH = -L../../ThirdParty/OpenNI2/Lib \
                       -L../../ThirdParty/OpenNI2/Redist/Linux/ARM \
	  	  
LDLIBS += -lOpenNI2 -lusb-1.0 -lGL -lGLU -lglut


CFLAGS=-O0 -DLINUX -g -Wall -std=c++11 -fpermissive
LDFLAGS += -Wl,-rpath=.

SOURCE = ../../src/*.cpp

TARGET = orbbecinteragesimple_rev1.0

all:
	$(CC) $(CFLAGS)  $(INCPATH) $(LIBPATH)  $(SOURCE) $(LDFLAGS) $(LDLIBS) -lpthread  -o $(TARGET)
	cp -r ../../ThirdParty/OpenNI2/Redist/Linux/ARM/*  ./
clean:
	rm -f *.o $(TARGET) *~

