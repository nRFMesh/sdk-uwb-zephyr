
#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

void main(void)
{
	LOG_INF("Hello world leds sample");

	int count = 0;
	while (1) {
		LOG_INF("leds> hello %d",count++);
		k_sleep(K_MSEC(5000));
	}
}
