# My Makefile


CC = gcc
#CSTD = c90
DEBUG = -g
#CFLAGS = -Wall --std=$(CSTD) $(DEBUG)
#CFLAGS = -Wall  $(DEBUG)

LIBS = -lwiringPi -lpthread

#a.out: main.c network
#	$(CC)  $(CFLAGS) -o $@ $< $(LIBS)


all: flyLinuxServer 

flyLinuxServer: main.o networkEnv.o
	$(CC)  $(CFLAGS)  main.o networkEnv.o -o flyLinuxServer $(LIBS)
	
main.o: main.c
	$(CC)  $(CFLAGS) -c main.c  
	
networkEnv.o: networkEnv.c
	$(CC)  $(CFLAGS) -c networkEnv.c 
	

clean:
	rm *.out -f
	rm flyLinuxServer -f
	rm *.o -f
