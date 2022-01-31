common_sources = sensirion_config.h sensirion_common.h sensirion_common.c hystrix-ec.h ccan_json_json.h ccan_json_json.c
i2c_sources = sensirion_i2c_hal.h sensirion_i2c.h sensirion_i2c.c
sgp40_sources = sgp40_i2c.h sgp40_i2c.c

i2c_implementation ?= sensirion_i2c_hal.c

CFLAGS = -Os -Wall -fstrict-aliasing -Wstrict-aliasing=1 -Wsign-conversion -fPIC -I.
LDLIBSOPTIONS=-Wl,-lgpiod
ifdef CI
	CFLAGS += -Werror
endif

.PHONY: all clean

all: hystrix-ec

hystrix-ec: clean
	$(CC) $(CFLAGS) -o $@  ${sgp40_sources} ${i2c_sources} \
	${i2c_implementation} ${common_sources} hystrix-ec.c ${LDLIBSOPTIONS} ${LDFLAGS}

clean:
		$(RM) hystrix-ec
