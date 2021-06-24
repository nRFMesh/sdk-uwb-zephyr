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

#include <meshposition.h>

#include <drivers/dw1000/deca_device_api.h>
#include <drivers/dw1000/deca_regs.h>
#include <drivers/dw1000/deca_spi.h>
#include <drivers/dw1000/port.h>

// zephyr includes
#include <zephyr.h>
#include <sys/printk.h>
#include <string>
#include <list>
#include <map>
#include <json.hpp>
using json = nlohmann::json;

#define LOG_LEVEL 3
#include <logging/log.h>
LOG_MODULE_REGISTER(main);

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

/* Frames used in the ranging process. See NOTE 2 below. */
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8_t tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E',0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define TX_ANT_DLY 16436

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
static uint8_t frame_seq_nb = 0;

#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];
static uint32_t status_reg = 0;

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
typedef unsigned long long uint64_t;
static uint64_t poll_tx_ts;
static uint64_t resp_rx_ts;
static uint64_t final_tx_ts;

/* Declaration of static functions. */
static void final_msg_set_ts(uint8_t *ts_field, uint64_t ts);

#define STACKSIZE 2048

void initiator_thread();
K_THREAD_DEFINE(initiator_main, STACKSIZE, initiator_thread, NULL, NULL, NULL, 99, 0, 0);

void initiator_thread(void)
{
    LOG_INF("initiator_thread> starting");

    mp_start(config);

    //expected to be low power saving measures to minimize RF listening time
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    dwt_setleds(1);
    k_yield();
    while (1) {
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb++;
        mp_request(tx_poll_msg,sizeof(tx_poll_msg));

        uint16_t frame_len;
        if(mp_receive(rx_buffer, frame_len))
        {
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {

                poll_tx_ts = get_tx_timestamp_u64();
                resp_rx_ts = get_rx_timestamp_u64();

                /* Compute final message transmission time. See NOTE 10 below. */
                uint32_t final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                
                /* Final TX timestamp is the transmission time we programmed 
                 * plus the TX antenna delay.
                 */
                final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

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

                if(mp_send_at(tx_final_msg, sizeof(tx_final_msg), final_tx_time))
                {
                    printk("success (%u)\n", frame_seq_nb);
                }
                else {
                    printk("\e[0;33m error - tx failed : status reg= 0x%08X\n\e[0m",status_reg);
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

        /* Execute a delay between ranging exchanges. */
        k_sleep(K_MSEC(RNG_DELAY_MS));
    }
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
static void final_msg_set_ts(uint8_t *ts_field, uint64_t ts)
{
    for (int i = 0; i < FINAL_MSG_TS_LEN; i++) {
        ts_field[i] = (uint8_t) ts;
        ts >>= 8;
    }
}
