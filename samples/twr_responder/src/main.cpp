/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   Double-sided two-way ranging (DS TWR) responder example code
 *
 *           This is a simple code example which acts as the responder in a 
 *           DS TWR distance measurement exchange. This application waits for 
 *           a "poll" message (recording the RX time-stamp of the poll) 
 *           expected from the "DS TWR initiator" example code (companion to 
 *           this application), and then sends a "response" message recording
 *           its TX time-stamp, after which it waits for a "final" message from
 *           the initiator to complete the exchange. The final message contains
 *           the remote initiator's time-stamps of poll TX, response RX and 
 *           final TX. With this data and the local time-stamps, (of poll RX,
 *           response TX and final RX), this example application works out a 
 *           value for the time-of-flight over-the-air and, thus, the 
 *           estimated distance between the two devices, which it writes 
 *           to console.
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 * Copyright 2019 (c) Frederic Mes, RTLOC.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include <meshposition.h>

#include <drivers/dw1000/deca_device_api.h>
#include <drivers/dw1000/deca_regs.h>
#include <drivers/dw1000/deca_spi.h>
#include <drivers/dw1000/port.h>

#include <stdio.h>
#include <string.h>

// zephyr includes
#include <zephyr.h>
#include <sys/printk.h>

#define LOG_LEVEL 3
#include <logging/log.h>
LOG_MODULE_REGISTER(main);

/* Example application name and version to display on console. */
#define APP_HEADER "\nDWM1001 & Zephyr\n"
#define APP_NAME "Example 5b - DS TWR RESP\n"
#define APP_VERSION "Version - 1.3\n"
#define APP_LINE "=================\n"

/* Default communication configuration. */
static dwt_config_t config = {5, DWT_PRF_64M, DWT_PLEN_128, DWT_PAC8, 9, 9, 1, DWT_BR_6M8, DWT_PHRMODE_EXT, (129) };

static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8_t rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define ALL_MSG_COMMON_LEN 10

/* Index to access some of the fields in the frames involved 
 * in the process.
 */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4

/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;
static uint8_t frame_seq_nb_rx = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed 
 * to handle.
 */
#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be 
 * examined at a debug breakpoint.
 */
static uint32_t status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion 
 * factor.
 * 1 uus = 512 / 499.2 usec and 1 usec = 499.2 * 128 dtu.
 */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */

/* This is the delay from Frame RX timestamp to TX reply timestamp used for 
 * calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration.
 */
#define POLL_RX_TO_RESP_TX_DLY_UUS 6000

/* This is the delay from the end of the frame transmission to the enable 
 * of the receiver, as programmed for the DW1000's wait for response feature.
 */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500

/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 10000

/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 30

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
uint64_t poll_rx_ts;
uint64_t resp_tx_ts;
uint64_t final_rx_ts;

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference so 
 * that it can be examined at a debug breakpoint.
 */
static double tof;
static double distance;

/* String used to display measured distance on console. */
char dist_str[32] = {0};

/* Declaration of static functions. */
static void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);


#define STACKSIZE 2048

void responder_thread();
K_THREAD_DEFINE(responder_main, STACKSIZE, responder_thread, NULL, NULL, NULL, 99, 0, 0);

void responder_thread(void)
{
    LOG_INF("responder_thread> starting");
    mp_start(config);

    k_yield();

    while (1) {
        dwt_setrxtimeout(0); // 0 : disable timeout
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
        while   ( !(    (status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & 
                        (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)
                    ))
        { };

        if (status_reg & SYS_STATUS_RXFCG) {    //Receiver FCS Good
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);//clear flag

            /* A frame has been received, read it into the local buffer. */
            uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= RX_BUFFER_LEN) {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            rx_buffer[ALL_MSG_SN_IDX] = 0;  //ignore sequence number in the next compare
            if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) {//is it poll (step1) msg ?

                /* Retrieve poll reception timestamp. */
                poll_rx_ts = get_rx_timestamp_u64();

                /* Retreive frame sequence number */
                memcpy(&frame_seq_nb_rx, &rx_buffer[2], 1);

                /* Set send time for response. See NOTE 9 below. */
                uint32_t resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(resp_tx_time);

                /* Set expected delay and timeout for final message reception.
                 * See NOTE 4 and 5 below.
                 */
                dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
                dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

                /* Write and send the response message. See NOTE 10 below.*/
                tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

                /* Zero offset in TX buffer. */
                dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); 
                
                /* Zero offset in TX buffer, ranging. */
                dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); 
                int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

                /* If dwt_starttx() returns an error, abandon this ranging 
                 * exchange and proceed to the next one. See NOTE 11 below.
                 */
                if (ret == DWT_ERROR) {
                    printk("err - tx_error1\n");
                    continue;
                }

                /* Poll for reception of expected "final" frame or error/timeout. 
                 * See NOTE 8 below.
                 */
                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & 
                    (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
                { };

                /* Increment frame sequence number after transmission of the 
                 * response message (modulo 256).
                 */
                frame_seq_nb++;

                if (status_reg & SYS_STATUS_RXFCG) {
                    /* Clear good RX frame event and TX frame sent in the 
                     * DW1000 status register.
                     */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

                    /* A frame has been received, read it into the local buffer. */
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                    if (frame_len <= RX_BUF_LEN) {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }

                    /* Check that the frame is a final message sent by "DS TWR
                     * initiator" example.
                     * As the sequence number field of the frame is not used 
                     * in this example, it can be zeroed to ease the validation
                     * of the frame.
                     */
                    rx_buffer[ALL_MSG_SN_IDX] = 0;
                    if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0) {

                        uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
                        uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                        double Ra, Rb, Da, Db;
                        int64 tof_dtu;

                        /* Retrieve response transmission and final reception 
                         * timestamps.
                         */
                        resp_tx_ts = get_tx_timestamp_u64();
                        final_rx_ts = get_rx_timestamp_u64();

                        /* Get timestamps embedded in the final message. */
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                        /* Compute time of flight. 32-bit subtractions give 
                         * correct answers even if clock has wrapped. 
                         * See NOTE 12 below.
                         */
                        poll_rx_ts_32 = (uint32_t)poll_rx_ts;
                        resp_tx_ts_32 = (uint32_t)resp_tx_ts;
                        final_rx_ts_32 = (uint32_t)final_rx_ts;
                        
                        Ra = (double)(resp_rx_ts - poll_tx_ts);
                        Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                        Da = (double)(final_tx_ts - resp_rx_ts);
                        Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                        tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                        tof = tof_dtu * DWT_TIME_UNITS;
                        distance = tof * SPEED_OF_LIGHT;

                        /* Display computed distance on console. */
                        sprintf(dist_str, "dist (%u): %3.2lf m\n",frame_seq_nb_rx, distance);
                        printk("%s", dist_str);
                    }
                }
                else {
                    printk("error - rx2 failed  %08x\n", status_reg);

                    /* Clear RX error/timeout events in the DW1000 
                     * status register.
                     */
                    dwt_write32bitreg(SYS_STATUS_ID, 
                        SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

                    /* Reset RX to properly reinitialise LDE operation. */
                    dwt_rxreset();
                }
            }
        }
        else {
            dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);//clear errors
            dwt_rxreset();
        }
    }
}


static void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts)
{
    *ts = 0;
    for (int i = 0; i < FINAL_MSG_TS_LEN; i++) {
        *ts += ts_field[i] << (i * 8);
    }
}
