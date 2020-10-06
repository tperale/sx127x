CONTIKI_PROJECT = test-sx1272

all: $(CONTIKI_PROJECT)

PLATFORMS_ONLY = zoul

ROOT = ../..
CONTIKI = $(ROOT)/contiki-ng

include $(CONTIKI)/Makefile.dir-variables
MODULES += $(ROOT)/src/ # SX127X Driver
MODULES += $(CONTIKI_NG_SERVICES_DIR)/shell
CFLAGS += -I/usr/arm-none-eabi/include

MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET

include $(CONTIKI)/Makefile.include
