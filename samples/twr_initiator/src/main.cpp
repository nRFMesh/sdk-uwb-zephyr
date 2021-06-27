/**
 * Copyright (c) 2019 - Frederic Mes, RTLOC
 * Copyright (c) 2015 - Decawave Ltd, Dublin, Ireland.
 * Copyright (c) 2021 - Home Smart Mesh
 * 
 * This file is part of Zephyr-DWM1001.
 *
 *   Zephyr-DWM1001 is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   Zephyr-DWM1001 is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with Zephyr-DWM1001.  If not, see <https://www.gnu.org/licenses/>.
 * 
 */

#include <string.h>

#include <meshposition.h>

#include <drivers/dw1000/deca_device_api.h>
#include <drivers/dw1000/deca_regs.h>
#include <drivers/dw1000/deca_spi.h>
#include <drivers/dw1000/port.h>

// zephyr includes
#include <zephyr.h>
#include <sys/printk.h>
#include <list>
#include <map>
#include <json.hpp>
using json = nlohmann::json;

#include <drivers/gpio.h>

#define LOG_LEVEL 3
#include <logging/log.h>
LOG_MODULE_REGISTER(main);

//[N] P0.13 => M_PIN17 => J7 pin 8
#define DEBUG_PIN_APP 	 13

//0.625 us per toggle
#define APP_SET 	gpio_pin_set(gpio_dev, DEBUG_PIN_APP, 1)	
#define APP_CLEAR 	gpio_pin_set(gpio_dev, DEBUG_PIN_APP, 0)

const struct device *gpio_dev;
void gpio_pin_init()
{
	gpio_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
	int ret = gpio_pin_configure(gpio_dev, DEBUG_PIN_APP, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("gpio_pin_configure() failed");
	}
}

/* Example application name and version to display on console. */
#define APP_HEADER "\nDWM1001 & Zephyr\n"
#define APP_NAME "Example 5a - DS TWR INIT\n"
#define APP_VERSION "Version - 1.4.0\n"
#define APP_VERSION_NUM 0x010400
#define APP_LINE "=================\n"

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default communication configuration. */
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
                        Used in RX only. */           
};

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Frames used in the ranging process. See NOTE 2 below. */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E',0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Length of the common part of the message (up to and including the 
 * function code, see NOTE 2 below). 
 */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) 
 * conversion factor.
 * 1 uus = 512 / 499.2 usec and 1 usec = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable 
 * of the receiver, as programmed for the DW1000's wait for response feature.
 */
#define POLL_TX_TO_RESP_RX_DLY_UUS 300

/* This is the delay from Frame RX timestamp to TX reply timestamp used 
 * for calculating/setting the DW1000's delayed TX function. 
 * This includes the frame length of approximately 2.66 ms with above 
 * configuration.
 */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 4000

/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 6000

/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 40

/* Time-stamps of frames transmission/reception, expressed in device time units.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);

#define STACKSIZE 2048

void initiator_thread();
K_THREAD_DEFINE(initiator_main, STACKSIZE, initiator_thread, NULL, NULL, NULL, 99, 0, 0);

void initiator_thread(void)
{
	gpio_pin_init();
	APP_CLEAR;
	APP_SET;
	APP_CLEAR;
    openspi();
    k_sleep(K_MSEC(100));//2 would be enough
    port_set_dw1000_slowrate();

    LOG_INF("initiator_thread> starting");
    printf("------------------------------------");
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        LOG_ERR("dwt_initialise failed");
        return;
    }
    port_set_dw1000_fastrate();

    dwt_configure(&config);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    //dwt_setpreambledetecttimeout(PRE_TIMEOUT);// NOTE: no use of preamble tmo's yet

    dwt_setleds(1);
    k_yield();
    while (1) {
        //APP_SET_CLEAR pulse1: 'tx 1st till rx resp' ; pulse2: 'tx final delayed till sent'
        /* Write frame data to DW1000 and prepare transmission. 
         * See NOTE 8 below.
         */
        uint32_t reg1 = dwt_read32bitreg(SYS_STATUS_ID);
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

        /* Zero offset in TX buffer. */
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); 

        /* Zero offset in TX buffer, ranging. */
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); 

        /* Start transmission, indicating that a response is expected so that 
         * reception is enabled automatically after the frame is sent and the 
         * delay set by dwt_setrxaftertxdelay() has elapsed.
         */
        APP_SET;
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        /* We assume that the transmission is achieved correctly, poll for 
         * reception of a frame or error/timeout. See NOTE 9 below. 
         */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & 
                (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        { };
        APP_CLEAR;
        /* Increment frame sequence number after transmission of the poll 
         * message (modulo 256).
         */
        frame_seq_nb++;

        if (status_reg & SYS_STATUS_RXFCG) 
        {
            uint32 frame_len;

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (frame_len <= RX_BUF_LEN) {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            /* Check that the frame is the expected response from the companion 
             * "DS TWR responder" example.
             * As the sequence number field of the frame is not relevant, 
             * it is cleared to simplify the validation of the frame.
             */
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {
                
                uint32 final_tx_time;
                int ret;

                /* Retrieve poll transmission and response reception timestamp. */
                poll_tx_ts = get_tx_timestamp_u64();
                resp_rx_ts = get_rx_timestamp_u64();

                /* Compute final message transmission time. See NOTE 10 below. */
                final_tx_time = (resp_rx_ts + 
                    (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                
                dwt_setdelayedtrxtime(final_tx_time);

                /* Final TX timestamp is the transmission time we programmed 
                 * plus the TX antenna delay.
                 */
                final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) +
                               TX_ANT_DLY;

                /* Write all timestamps in the final message. 
                 * See NOTE 11 below.
                 */
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

                /* Write and send final message.
                 * See NOTE 8 below.
                 */
                tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

                /* Zero offset in TX buffer. */
                dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); 
                
                /* Zero offset in TX buffer, ranging. */
                dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1); 
                APP_SET;
                ret = dwt_starttx(DWT_START_TX_DELAYED);
                if (ret == DWT_SUCCESS) {
                    /* Poll DW1000 until TX frame sent event set. 
                     * See NOTE 9 below.
                     */
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                    { };
                    APP_CLEAR;

                    /* Clear TXFRS event. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                    printk("success (%u)\n", frame_seq_nb);

                    /* Increment frame sequence number after transmission of 
                     * the final message (modulo 256).
                     */
                    frame_seq_nb++;
                }
                else {
                    APP_CLEAR;
                    printk("\e[0;33m error - tx failed : status reg= 0x%08lX\n\e[0m",status_reg);
                    mp_status_print(status_reg);
                }
            }
        }
        else {
            printk("timeout: (rx)\n");

            /* Clear RX error/timeout events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, 
                              SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

            /* Reset RX to properly reinitialise LDE operation. */
            dwt_rxreset();
        }

        APP_CLEAR;
        uint32_t reg2 = dwt_read32bitreg(SYS_STATUS_ID);
        printk("initiator> sequence(%u) over; reg1 = 0x%08x ; reg2 = 0x%08x\n",frame_seq_nb,reg1,reg2);
        mp_status_print(reg2);
        /* Execute a delay between ranging exchanges. */
        k_sleep(K_MSEC(RNG_DELAY_MS));
    }
}

/*! --------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, 
 *            for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;

    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! --------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits,
 *        for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;

    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! --------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the 
 *        given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    for (int i = 0; i < FINAL_MSG_TS_LEN; i++) {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}
