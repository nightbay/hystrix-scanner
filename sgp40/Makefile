common_sources = sensirion_config.h sensirion_common.h sensirion_common.c
i2c_sources = sensirion_i2c_hal.h sensirion_i2c.h sensirion_i2c.c
sgp40_sources = sgp40_i2c.h sgp40_i2c.c

i2c_implementation ?= sensirion_i2c_hal.c

CFLAGS = -Os -Wall -fstrict-aliasing -Wstrict-aliasing=1 -Wsign-conversion -fPIC -I.

ifdef CI
    CFLAGS += -Werror
endif

.PHONY: all clean

all: hystrix-voc

hystrix-voc: clean
	$(CC) $(CFLAGS) -o $@  ${sgp40_sources} ${i2c_sources} \
		${i2c_implementation} ${common_sources} hystrix-voc.c

clean:
	$(RM) hystrix-voc
