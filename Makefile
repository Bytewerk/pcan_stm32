#DEVICE          = stm32f407vg
DEVICE          = stm32f407ve
OPENCM3_DIR     = ./libopencm3
OBJS            += main.o can.o can_queue.o ppro_protocol.o ppro_usb.o ppro_usb_descr.o systime.o external/pcan_usbpro_sizeof_rec.o

CFLAGS          += -Os -ggdb3 -DDISABLE_VBUS_SENSE
CPPFLAGS	    += -MD
LDFLAGS         += -static -nostartfiles
LDLIBS          += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

.PHONY: clean all

all: binary.elf binary.hex

clean:
	$(Q)$(RM) -rf binary.* *.o

include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk
