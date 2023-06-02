NAME=main

GCC=/mnt/ssd/projects/lichee/lichee_nano/buildroot-2020.11.1/output/host/bin/arm-buildroot-linux-uclibcgnueabi-gcc
GPP=/mnt/ssd/projects/lichee/lichee_nano/buildroot-2020.11.1/output/host/bin/arm-buildroot-linux-uclibcgnueabi-g++
GCC=gcc

LIB=/home/fademike/Yandex.Disk/ubuntu/project/linux/mavlink_drone/mavlink/common/
STD=-std=c99

CCFLAGS=-${STD} -I ${LIB}

SRC=$(wildcard *.c)
OBJ=$(SRC:%.c=%.o)
TRASH=*.o

all: $(NAME)

clean:
	rm -f $(TRASH)
	rm -f $(NAME)

%.o: %.c
	$(GCC) $(CCFLAGS) -c $<

$(NAME): $(OBJ)
	$(GCC) $(OBJ) -o $(NAME)

.PHONY: all
