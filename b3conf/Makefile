obj= b3conf.o
src= $(obj:.c=.o)

CFLAGS= -g

all: b3conf

b3test: $(obj)
	$(CC) $(CFLAGS) -o $@ $(obj)

b3test.c: b3packet.h

clean:
	rm -f b3conf *~ *.o
