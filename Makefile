common_sources = hystrix-ec.h ccan_json_json.h ccan_json_json.c

CFLAGS = -Os -Wall -fstrict-aliasing -Wstrict-aliasing=1 -Wsign-conversion -fPIC -I.

ifdef CI
	CFLAGS += -Werror
endif

.PHONY: all clean

all: hystrix-ec hystrix-pm hystrix-voc

hystrix-voc:
	cd sgp40 && make all

hystrix-pm:
	cd embedded-uart-sps && make all
	
hystrix-ec: clean
	$(CC) $(CFLAGS) -o $@   \
	${common_sources} hystrix-ec.c  ${LDFLAGS}

clean:
		$(RM) hystrix-ec
		make -Csgp40 clean
		make -C embedded-uart-sps clean
