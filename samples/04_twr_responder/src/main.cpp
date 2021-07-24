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
#include <drivers/dw1000/deca_spi.h>
#include <drivers/dw1000/port.h>

#include <stdio.h>
#include <string.h>

// zephyr includes
#include <zephyr.h>
#include <sys/printk.h>

#include <drivers/gpio.h>

#define LOG_LEVEL 3
#include <logging/log.h>
//note that log level info produces more rx errors, not clear how to log tasks influences the runtime
LOG_MODULE_REGISTER(main, LOG_LEVEL_ERR);

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

/* Default communication configuration. */
static dwt_config_t config = {5, DWT_PRF_64M, DWT_PLEN_128, DWT_PAC8, 9, 9, 1, DWT_BR_6M8, DWT_PHRMODE_EXT, (129) };

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion 
 * factor.
 * 1 uus = 512 / 499.2 usec and 1 usec = 499.2 * 128 dtu.
 */
#define UUS_TO_DWT_TIME 65536

#define POLL_RX_TO_RESP_TX_DLY_UUS 1000
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

#define STACKSIZE 2048

void responder_thread();
K_THREAD_DEFINE(responder_main, STACKSIZE, responder_thread, NULL, NULL, NULL, 99, 0, 0);

void responder_thread(void)
{
	gpio_pin_init();
	APP_CLEAR;
	APP_SET;
	APP_CLEAR;
    LOG_INF("responder_thread> starting");
    mp_start(config);

    k_yield();
    uint32_t sequence = 0;
    while (1) {
        //APP_SET_CLEAR 
        // - pulse1: 'request_at : start request resp_2 tx till sent' ; 
        // - pulse2: 'receive : pending for receive final_3'
        // - pulse3: 'computing distance'
        uint32_t reg1 = mp_get_status();
        LOG_INF("responder> sequence(%u) starting ; statusreg = 0x%08x",sequence,reg1);
        mp_rx_now();
        msg_header_t rx_poll_msg;
        if(mp_receive(msg_id_t::twr_1_poll,rx_poll_msg)){
            uint64_t poll_rx_ts = get_rx_timestamp_u64();

            mp_rx_after_tx(RESP_TX_TO_FINAL_RX_DLY_UUS,10000);
            
            uint32_t resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;

            msg_header_t tx_resp_msg = {
                    msg_id_t::twr_2_resp, (uint8_t)(rx_poll_msg.header.sequence + 1),
                    rx_poll_msg.header.dest , rx_poll_msg.header.source,0
                };
            APP_SET;
            if(mp_request_at((uint8_t*)&tx_resp_msg, sizeof(msg_header_t), resp_tx_time)){
                APP_CLEAR;
                msg_twr_final_t final_msg;
                APP_SET;
                if(mp_receive(msg_id_t::twr_3_final,final_msg)){
                    APP_CLEAR;
                    k_sleep(K_USEC(10));
                    APP_SET;
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
                    APP_CLEAR;
                    char dist_str[30];
                    sprintf(dist_str, "responder> dist (%u): %3.2lf m\n",rx_poll_msg.header.sequence, distance);
                    printk("%s", dist_str);
                }else{
                    LOG_WRN("mp_receive(twr_3_final) fail at rx frame %u",rx_poll_msg.header.sequence);
                    APP_CLEAR;
                }
            }else{
                APP_CLEAR;
                LOG_WRN("mp_request_at(twr_2_resp) fail at rx frame %u",rx_poll_msg.header.sequence);
            }
        }

        uint32_t reg2 = mp_get_status();
        LOG_INF("responder> sequence(%u) over; statusreg = 0x%08x",sequence,reg2);
        sequence++;
    }
}
