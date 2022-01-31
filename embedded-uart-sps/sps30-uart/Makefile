# See user_config.inc for build customizations
-include user_config.inc
include default_config.inc

.PHONY: all clean

all: hystrix-pm

hystrix-pm: clean
	$(CC) $(CFLAGS) -o $@ ${sps30_uart_sources} ${uart_sources} ${sps30_uart_dir}/hystrix-pm.c

clean:
	$(RM) hystrix-pm
