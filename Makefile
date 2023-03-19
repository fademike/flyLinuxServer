
GCC=/mnt/ssd/projects/lichee/lichee_nano/buildroot-2020.11.1/output/host/bin/arm-buildroot-linux-uclibcgnueabi-gcc
GPP=/mnt/ssd/projects/lichee/lichee_nano/buildroot-2020.11.1/output/host/bin/arm-buildroot-linux-uclibcgnueabi-g++
#GCC=gcc

# LIB=/home/fademike/Programs/mavlink/examples/mygen/out_cpp_common/common/
LIB=/home/fademike/Yandex.Disk/ubuntu/project/linux/mavlink_drone/mavlink/common/
# STD=-std=c99
STD=-std=c99 -D_XOPEN_SOURCE=600 -D_POSIX_C_SOURCE=200112L

all: main

main: main.o mavlink_handler.o ModemControl.o nRF24.o spi.o network.o params.o
	${GCC} main.o mavlink_handler.o ModemControl.o nRF24.o spi.o network.o params.o -o main

main.o: main.c
	${GCC} ${STD} -I ../../include/common -I ${LIB} -c main.c

mavlink_handler.o: mavlink_handler.c
	${GCC} ${STD} -I ../../include/common -I ${LIB} -c mavlink_handler.c

ModemControl.o: ModemControl.c
	${GCC} -c ModemControl.c
network.o: network.c
	${GCC} -c network.c
params.o: params.c
	${GCC}  -I ${LIB} -c params.c
nRF24.o: nRF24.c
	${GCC} -c nRF24.c
spi.o: spi.c
	${GCC} -c spi.c


clean:
	rm ./*.o
	rm main
