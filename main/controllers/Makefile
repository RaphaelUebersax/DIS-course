CC = gcc
AR = ar

CFLAGS = -O3 -Wall -I.
LDFLAGS = -L. -lhungarian

all: libhungarian.a 


libhungarian.a: hungarian.o
	$(AR) cr $@ hungarian.o

%.o: %.c %.h
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f *.o *.a 
