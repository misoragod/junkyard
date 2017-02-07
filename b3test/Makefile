obj= b3test.o js.o MadgwickAHRS.o
src= $(obj:.c=.o)

CFLAGS= -g

all: b3test

b3test: $(obj)
	$(CC) $(CFLAGS) -o $@ $(obj) -lm -lpthread

b3test.c, MadgwickAHRS.c: MadgwickAHRS.h

clean:
	rm -f b3test *~ *.o
