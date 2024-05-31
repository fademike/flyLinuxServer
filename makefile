
NAME=main

#GCC_DIR=/home/fademike/Programs/STMicroelectronics/stm32cubeide_1.7.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.9-2020-q2-update.linux64_2.0.0.202105311346/tools/bin
GCC_DIR=/usr/bin
GCC = $(GCC_DIR)/gcc
OBJCOPY = $(GCC_DIR)/objcopy
SIZE = $(GCC_DIR)/size
OUTDIR = out

SRCDIR = src

INC = -I ./src
INC += -I ./src/mavlink/common

#OPTIMIZATION=-O0 -g3 -DDEBUG
OPTIMIZATION=-Os

CCFLAGS = $(OPTIMIZATION) -Wall -c 

LDFLAGS= -lm
#LDFLAGS+=-Wl,--print-memory-usage
#LDFLAGS+=Wl,-Map=$(OUTDIR)/output.map

FILES_C := $(foreach dir,$(SRCDIR),$(wildcard $(dir)/*.c)) 
FILES_S := $(foreach dir,$(SRCDIR),$(wildcard $(dir)/*.s)) 

OBJECTS = $(patsubst %.c, $(OUTDIR)/%.o,${FILES_C})
OBJECTS += $(patsubst %.s, $(OUTDIR)/%.o,${FILES_S})

all: $(NAME)

clean:
	-rm -Rf $(OUTDIR)/

program: $(NAME)
	./$(NAME) -D /dev/ttyUSB0 -P ../fly_drone/stm32f103_FlyBoard/out/stm32f103_FlyBoard.bin
run: $(NAME)
	./$(NAME)

$(OUTDIR)/%.o: %.c
	@-mkdir -p $(dir $@)
	@$(GCC) $(CCFLAGS) $(INC) -c $< -o $@
$(OUTDIR)/%.o: %.s
	@-mkdir -p  $(dir $@)
	@$(GCC) $(CCFLAGS) $(INC) -c $< -o $@
	
$(NAME): $(OBJECTS)
	@$(GCC) $(INC) $(LDFLAGS) $(OBJECTS) -o $@
	@echo link $(NAME) OK
	@$(SIZE) $(NAME)


	
.PHONY: all
