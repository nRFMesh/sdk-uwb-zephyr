
#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>
#include <drivers/gpio.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define LED0_NODE 	DT_ALIAS(led0)
#define LED0		DT_GPIO_LABEL(LED0_NODE, gpios)
#define LED0_PIN	DT_GPIO_PIN(LED0_NODE, gpios)

#define LED1_NODE 	DT_ALIAS(led1)
#define LED1		DT_GPIO_LABEL(LED1_NODE, gpios)
#define LED1_PIN	DT_GPIO_PIN(LED1_NODE, gpios)

#define LED2_NODE 	DT_ALIAS(led2)
#define LED2		DT_GPIO_LABEL(LED2_NODE, gpios)
#define LED2_PIN	DT_GPIO_PIN(LED2_NODE, gpios)

#define LED3_NODE 	DT_ALIAS(led3)
#define LED3		DT_GPIO_LABEL(LED3_NODE, gpios)
#define LED3_PIN	DT_GPIO_PIN(LED3_NODE, gpios)



void main(void)
{
	LOG_INF("Hello world leds sample");

	const struct device *dev = device_get_binding(LED0);

	gpio_pin_configure(dev, LED0_PIN, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure(dev, LED1_PIN, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure(dev, LED2_PIN, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure(dev, LED3_PIN, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set(dev, LED0_PIN, 0);
	gpio_pin_set(dev, LED1_PIN, 0);
	gpio_pin_set(dev, LED2_PIN, 0);
	gpio_pin_set(dev, LED3_PIN, 0);
	k_sleep(K_MSEC(1000));
	gpio_pin_set(dev, LED0_PIN, 1);
	gpio_pin_set(dev, LED1_PIN, 1);
	gpio_pin_set(dev, LED2_PIN, 1);
	gpio_pin_set(dev, LED3_PIN, 1);
	k_sleep(K_MSEC(1000));

	int count = 0;
	while (1) {
		LOG_INF("leds> hello %d",count++);
		gpio_pin_set(dev, LED0_PIN, 0);
		k_sleep(K_MSEC(1000));
		gpio_pin_set(dev, LED0_PIN, 1);
		gpio_pin_set(dev, LED1_PIN, 0);
		k_sleep(K_MSEC(1000));
		gpio_pin_set(dev, LED1_PIN, 1);
		gpio_pin_set(dev, LED2_PIN, 0);
		k_sleep(K_MSEC(1000));
		gpio_pin_set(dev, LED2_PIN, 1);
		gpio_pin_set(dev, LED3_PIN, 0);
		k_sleep(K_MSEC(1000));
		gpio_pin_set(dev, LED3_PIN, 1);
	}
}
