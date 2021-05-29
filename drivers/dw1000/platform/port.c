/*! ----------------------------------------------------------------------------
 * @file    port.c
 * @brief   HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 * Copyright 2019 (c) Frederic Mes, RTLOC. 
 *
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include <drivers/dw1000/port.h>
#include <drivers/dw1000/deca_device_api.h>
#include <drivers/dw1000/deca_spi.h>

// zephyr includes
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <soc.h>
#include <hal/nrf_gpiote.h>
#include <drivers/gpio.h>

static const struct device * gpio_dev;
static struct gpio_callback gpio_cb;

/****************************************************************************//**
 *
 *                              APP global variables
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                  Port private variables and function prototypes
 *
 *******************************************************************************/
static volatile uint32_t signalResetDone;

/****************************************************************************//**
 *
 *                              Time section
 *
 *******************************************************************************/

/* @fn    usleep
 * @brief precise usleep() delay
 * */
#pragma GCC optimize ("O0")
int usleep(unsigned long usec)
{
    int i,j;
#pragma GCC ivdep
    for(i=0;i<usec;i++)
    {
#pragma GCC ivdep
        for(j=0;j<2;j++)
        {
            // __NOP();
            // __NOP();
        }
    }
    return 0;
}

/****************************************************************************//**
 *
 *                              END OF Time section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                              Configuration section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                          End of configuration section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                          DW1000 port section
 *
 *******************************************************************************/



/* @fn      port_set_dw1000_slowrate
 * @brief   set 2MHz
 * */
void port_set_dw1000_slowrate(void)
{
    set_spi_speed_slow();
}

/* @fn      port_set_dw1000_fastrate
 * @brief   set 8MHz
 * */
void port_set_dw1000_fastrate(void)
{
    //TODO
    set_spi_speed_fast();
}


/****************************************************************************//**
 *
 *                          End APP port section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                              END OF IRQ section
 *
 *******************************************************************************/

/* DW1000 IRQ handler definition. */

#define GPIO_PIN     19
#define GPIO_FLAGS   (GPIO_PULL_UP | GPIO_ACTIVE_LOW)

/*! ---------------------------------------------------------------------------
 * @fn port_set_deca_isr()
 *
 * @brief This function is used to install the handling function for DW1000 IRQ.
 *
 * NOTE:
 *
 * @param deca_isr function pointer to DW1000 interrupt handler to install
 *
 * @return none
 */
void port_set_deca_isr(port_deca_isr_t deca_isr)
{
    printk("%s: Binding to GPIO0 and pin %d\n", __func__, GPIO_PIN);
    gpio_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
    if (!gpio_dev) {
        printk("error\n");
        return;
    }

    /* Decawave interrupt */
    gpio_pin_configure(gpio_dev, GPIO_PIN, (GPIO_INPUT | GPIO_FLAGS));

    gpio_init_callback(&gpio_cb, (gpio_callback_handler_t)(deca_isr), BIT(GPIO_PIN));

    gpio_add_callback(gpio_dev, &gpio_cb);

    gpio_pin_interrupt_configure(gpio_dev, GPIO_PIN, GPIO_INT_EDGE_RISING);
}

/****************************************************************************//**
 *
 *******************************************************************************/
