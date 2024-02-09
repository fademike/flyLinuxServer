

cmake_minimum_required(VERSION 3.10)

# set the project name
project(main)

set(tutorial_SRCS cb.c mavlink_handler.c network.c params.c spi.c main.c ModemControl.c nRF24.c si4463.c tty.c program_pack.c)

# add the executable
add_executable(main ${tutorial_SRCS})

include_directories(mavlink/common)

