CONTIKI_PROJECT = node coordinator
all: $(CONTIKI_PROJECT)

TARGET = zoul
BOARD = remote-revb

ROOT = ../..
CONTIKI = $(ROOT)/contiki-ng

include $(CONTIKI)/Makefile.dir-variables
MODULES += ../src/ # SX127X Driver
MODULES += $(CONTIKI_NG_SERVICES_DIR)/shell
CFLAGS += -I/usr/arm-none-eabi/include

MAKE_MAC = MAKE_MAC_TSCH
MAKE_ROUTING = MAKE_ROUTING_RPL_LITE

include $(CONTIKI)/Makefile.include
