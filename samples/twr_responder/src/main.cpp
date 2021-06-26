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

/* Default communication configuration. */
static dwt_config_t config = {5, DWT_PRF_64M, DWT_PLEN_128, DWT_PAC8, 9, 9, 1, DWT_BR_6M8, DWT_PHRMODE_EXT, (129) };

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion 
 * factor.
 * 1 uus = 512 / 499.2 usec and 1 usec = 499.2 * 128 dtu.
 */
#define UUS_TO_DWT_TIME 65536

#define POLL_RX_TO_RESP_TX_DLY_UUS 6000
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

#define STACKSIZE 2048

void responder_thread();
K_THREAD_DEFINE(responder_main, STACKSIZE, responder_thread, NULL, NULL, NULL, 99, 0, 0);

void responder_thread(void)
{
    LOG_INF("responder_thread> starting");
    mp_start(config);

    k_yield();
    uint32_t sequence = 0;
    while (1) {
        LOG_INF("starting sequence %u",sequence++);
        mp_rx_now();

        msg_header_t rx_poll_header;
        if(!mp_receive(msg_id_t::twr_1_poll,rx_poll_header)){
            continue;
        }
        uint64_t poll_rx_ts = get_rx_timestamp_u64();

        mp_rx_after_tx(RESP_TX_TO_FINAL_RX_DLY_UUS);
        
        uint32_t resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;

        msg_header_t tx_resp_header = {msg_id_t::twr_2_resp, (uint8_t)(rx_poll_header.sequence + 1), rx_poll_header.dest , rx_poll_header.source};
        if(!mp_send_at((uint8_t*)&tx_resp_header, sizeof(msg_header_t), resp_tx_time,DWT_RESPONSE_EXPECTED)){
            LOG_ERR("mp_send_at(twr_2_resp) fail at rx frame %u",rx_poll_header.sequence);
            continue;
        }

        msg_twr_final_t final_msg;
        if(!mp_receive(msg_id_t::twr_3_final,final_msg)){
            LOG_ERR("mp_receive(twr_3_final) fail at rx frame %u",rx_poll_header.sequence);
            continue;
        }

        uint64_t resp_tx_ts  = get_tx_timestamp_u64();
        uint64_t final_rx_ts = get_rx_timestamp_u64();

        uint32_t poll_rx_ts_32  = (uint32_t)poll_rx_ts;
        uint32_t resp_tx_ts_32  = (uint32_t)resp_tx_ts;
        uint32_t final_rx_ts_32 = (uint32_t)final_rx_ts;
        
        double Ra = (double)(final_msg.resp_rx_ts - final_msg.poll_tx_ts);
        double Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
        double Da = (double)(final_msg.final_tx_ts - final_msg.resp_rx_ts);
        double Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
        int64_t tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

        double tof = tof_dtu * DWT_TIME_UNITS;
        double distance = tof * SPEED_OF_LIGHT;

        printk("frame %u ; dist = %3.2lf m\n",rx_poll_header.sequence, distance);//for printing float is enough
        printf("frame %u ; dist = %3.2f m\n",rx_poll_header.sequence, (float)distance);//for printing float is enough
    }
}
