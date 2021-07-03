/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   RX using SNIFF mode example code
 *
 * @attention
 *
 * Copyright 2016 (c) Decawave Ltd, Dublin, Ireland.
 * Copyright 2019 (c) Frederic Mes, RTLOC.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include <zephyr.h>

#include <drivers/dw1000/deca_device_api.h>
#include <drivers/dw1000/deca_regs.h>
#include <drivers/dw1000/deca_spi.h>
#include <drivers/dw1000/port.h>

// zephyr includes
#include <zephyr.h>
#include <sys/printk.h>

#define STACKSIZE 1024
#define PRIORITY 99
#define DELAY_TIME   K_MSEC(1000)

#define LOG_LEVEL 3
#include <logging/log.h>
LOG_MODULE_REGISTER(main);

/* Example application name and version to display on console. */
#define APP_NAME "SIMPLE RX v1.3\n"

static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_EXT, /* PHY header mode. */
    (129)            /* SFD timeout (preamble length + 1 + SFD length - PAC size).
                      * Used in RX only. */
};

/* Buffer to store received frame. See NOTE 1 below. */
#define FRAME_LEN_MAX 127
static uint8 rx_buffer[FRAME_LEN_MAX];

/* Hold copy of status register state here for reference so that it can be 
   examined at a debug breakpoint.
*/
static uint32 status_reg = 0;

/* Hold copy of frame length of frame received (if good) so that it can be 
   examined at a debug breakpoint.
*/
static uint16 frame_len = 0;

void sniff_ms(int time)
{
    int start = k_uptime_get();
    int finish = start + time;

    while (k_uptime_get() < finish) {
        for (int i = 0 ; i < FRAME_LEN_MAX; i++ )
        {
            rx_buffer[i] = 0;
        }
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & 
                 (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) { /* spin */ };

        if (status_reg & SYS_STATUS_RXFCG) {
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= FRAME_LEN_MAX) {
                dwt_readrxdata(rx_buffer, frame_len, 0);
                int time_stamp = k_uptime_get();
                printf("%d> ",time_stamp);
                int i;
                for (i = 0 ; i < frame_len; i++ )
                {
                    printf("%02x ",rx_buffer[i]);
                }
                printf("\r\n");
            }
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
        }
        else {
            printf("0x%04lX\r\n",status_reg);
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
    }

}

void main_thread(void * id, void * unused1, void * unused2)
{
    printk("%s\n", __func__);
    printk(APP_NAME);
    openspi();

    port_set_dw1000_slowrate();

    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR) {
        printk("INIT FAILED");
        k_sleep(K_MSEC(500)); 
        while (1) { /* spin */};
    }
    port_set_dw1000_fastrate();

    dwt_configure(&config);

    printf("configured : chan %u, rxCode %u, nsSFD %u\r\n",
                config.chan, config.rxCode, config.nsSFD);

    dwt_setleds(1);

    k_yield();

    while (1) {
        sniff_ms(10000);
    }
}

K_THREAD_DEFINE(main_id, STACKSIZE, main_thread, 
                NULL, NULL, NULL, PRIORITY, 0, 0);


/*****************************************************************************
 * NOTES:
 *
 * 1. The device ID is a hard coded constant in the blink to keep the example 
 *    simple but for a real product every device should have a unique ID.
 *    For development purposes it is possible to generate a DW1000 unique ID 
 *    by combining the Lot ID & Part Number values programmed into the
 *    DW1000 during its manufacture. However there is no guarantee this will 
 *    not conflict with someone else's implementation. We recommended that
 *    customers buy a block of addresses from the IEEE Registration Authority 
 *    for their production items. See "EUI" in the DW1000 User Manual.
 * 2. In this example, LDE microcode is not loaded upon calling dwt_initialise().
 *    This will prevent the IC from generating an RX timestamp. 
 *    If time-stamping is required, DWT_LOADUCODE parameter should be used. 
 *    See two-way ranging examples (e.g. examples 5a/5b).
 * 3. In a real application, for optimum performance within regulatory limits, 
 *    it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in 
 *    the target system or the DW1000 OTP memory.
 * 4. dwt_writetxdata() takes the full size of tx_msg as a parameter but only 
 *    copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our tx_msg could 
 *    be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of
 *    the frame to dwt_writetxdata()).
 * 5. We use polled mode of operation here to keep the example as simple as 
 *    possible but the TXFRS status event can be used to generate an interrupt.
 *    Please refer to DW1000 User Manual for more details on "interrupts".
 * 6. The user is referred to DecaRanging ARM application (distributed with 
 *    EVK1000 product) for additional practical example of usage, and to the
 *    DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************/
