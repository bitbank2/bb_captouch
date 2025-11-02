CFLAGS=-D__LINUX__ -Wall -O2 -I../../
LIBS=-lbb_captouch -lgpiod 

all: touch_demo

touch_demo: main.o
	$(CXX) main.o $(LIBS) -o touch_demo 

main.o: main.cpp
	$(CXX) $(CFLAGS) -c main.cpp

clean:
	rm -rf *.o touch_demo
