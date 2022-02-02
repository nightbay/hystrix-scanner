common_sources = hystrix-ec.h ccan_json_json.h ccan_json_json.c

CFLAGS = -Os -Wall -fstrict-aliasing -Wstrict-aliasing=1 -Wsign-conversion -fPIC -I.

ifdef CI
	CFLAGS += -Werror
endif

.PHONY: all clean

all: hystrix-ec hystrix-pm hystrix-voc

hystrix-voc:
	$(MAKE) -C sgp40 all

hystrix-pm:
	$(MAKE) -C embedded-uart-sps all
	
hystrix-ec: clean
	$(CC) $(CFLAGS) -o $@   \
	${common_sources} hystrix-ec.c  ${LDFLAGS}

clean:
		$(RM) hystrix-ec
		$(MAKE) -C sgp40 clean
		$(MAKE) -C embedded-uart-sps clean
