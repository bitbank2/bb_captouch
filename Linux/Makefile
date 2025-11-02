CFLAGS=-c -Wall -O2 -ggdb -D__LINUX__ -I../src
LIBS = -lm -lpthread

all: libbb_captouch.a

libbb_captouch.a: bb_captouch.o
	ar -rc libbb_captouch.a bb_captouch.o ;\
	sudo cp libbb_captouch.a /usr/local/lib ;\
	sudo cp ../src/bb_captouch.h /usr/local/include

bb_captouch.o: ../src/bb_captouch.cpp ../src/bb_captouch.h ../src/rpi_io.inl
	$(CXX) $(CFLAGS) ../src/bb_captouch.cpp

clean:
	rm *.o libbb_captouch.a
