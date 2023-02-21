OBJS = main.o
OPENCM3_DIR := libopencm3

# Our output name

BINARY = main
LDSCRIPT = stm32f1.ld

# Using the stm32f0 series chip

LIBNAME     = opencm3_stm32f1
DEFS        += -DSTM32F1

# Target-specific flags

FP_FLAGS    ?= -msoft-float
ARCH_FLAGS   = -mthumb -mcpu=cortex-m3 $(FP_FLAGS) -mfix-cortex-m3-ldrd

################################################################################
# OpenOCD specific variables

OOCD         = $(shell command -v openocd)
OOCD_IF      = interface/stlink-v2.cfg
OOCD_TG      = target/stm32f1x.cfg
STFLASH      = $(shell command -v st-flash)

################################################################################
# Black Magic Probe specific variables
# Set the BMP_PORT to a serial port and then BMP is used for flashing
BMP_PORT	?=

################################################################################
# texane/stlink specific variables
#STLINK_PORT	?= :4242

###############################################################################
# Executables

PREFIX		?= arm-none-eabi-

CC		:= $(PREFIX)gcc
CXX		:= $(PREFIX)g++
LD		:= $(PREFIX)gcc
AR		:= $(PREFIX)ar
AS		:= $(PREFIX)as
OBJCOPY	:= $(PREFIX)objcopy
OBJDUMP	:= $(PREFIX)objdump
GDB		:= $(PREFIX)gdb
STFLASH	= $(shell which st-flash)
STYLECHECK	:= /checkpatch.pl
STYLECHECKFLAGS	:= --no-tree -f --terse --mailback
STYLECHECKFILES	:= $(shell find . -name '*.[ch]')
OPT		:= -Os
DEBUG		:= -ggdb3
CSTD		?= -std=c99

###############################################################################
# C flags

TGT_CFLAGS  += $(OPT) $(CSTD) $(DEBUG)
TGT_CFLAGS  += $(ARCH_FLAGS)
TGT_CFLAGS  += -Wextra -Wshadow -Wimplicit-function-declaration
TGT_CFLAGS  += -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes
TGT_CFLAGS  += -fno-common -ffunction-sections -fdata-sections

###############################################################################
# C++ flags

TGT_CXXFLAGS	+= $(OPT) $(CXXSTD) $(DEBUG)
TGT_CXXFLAGS	+= $(ARCH_FLAGS)
TGT_CXXFLAGS	+= -Wextra -Wshadow -Wredundant-decls  -Weffc++
TGT_CXXFLAGS	+= -fno-common -ffunction-sections -fdata-sections

###############################################################################
# C & C++ preprocessor common flags

TGT_CFLAGS    += -MD
TGT_CFLAGS    += -Wall -Wundef
TGT_CFLAGS    += $(DEFS)

###############################################################################
# Linker flags

TGT_LDFLAGS     += --static -nostartfiles
TGT_LDFLAGS     += -T$(LDSCRIPT)
TGT_LDFLAGS     += $(ARCH_FLAGS) $(DEBUG)
TGT_LDFLAGS     += -Wl,-Map=$(*).map -Wl,--cref
TGT_LDFLAGS     += -Wl,--gc-sections

###############################################################################
# Used libraries

DEFS        += -I$(OPENCM3_DIR)/include
LDFLAGS     += -L$(OPENCM3_DIR)/lib
LDLIBS      += -l$(LIBNAME)
LDLIBS      += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

all: size

$(OPENCM3_DIR)/lib/lib$(LIBNAME).a:

ifeq (,$(wildcard $@))
	$(warning $(LIBNAME).a not found, attempting to rebuild in $(OPENCM3_DIR))
	$(MAKE) -C $(OPENCM3_DIR)
endif

size: $(BINARY).size
elf: $(BINARY).elf
bin: $(BINARY).bin
map: $(BINARY).map

GENERATED_BINARIES=$(BINARY).elf $(BINARY).bin $(BINARY).map

%.bin: %.elf

	$(OBJCOPY) -Obinary $(*).elf $(*).bin

%.elf %.map: $(OBJS) $(LDSCRIPT)

	$(LD) $(TGT_LDFLAGS) $(LDFLAGS) $(OBJS) $(LDLIBS) -o $(*).elf

%.o: %.c $(OPENCM3_DIR)/lib/lib$(LIBNAME).a

	$(CC) $(TGT_CFLAGS) $(CFLAGS)  -o $(*).o -c $(*).c

%.size: %.elf

	@$(SIZE) -d $(*).elf | tee $(*).size

flash: $(BINARY).elf

	@printf "  FLASH  $<\n"

	$(OOCD) -f $(OOCD_IF) -f $(OOCD_TG) -c "program $(BINARY).elf verify reset exit"

flash-stlink: $(BINARY).elf

	@printf "  FLASH  $<\n"

	$(STFLASH) write $(BINARY).bin 0x8000000

reset:

	$(STFLASH)

clean:

	$(RM) $(GENERATED_BINARIES) generated.* $(OBJS) $(OBJS:%.o=%.d)

.PHONY: clean flash
