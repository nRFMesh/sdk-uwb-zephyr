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

#include <drivers/gpio.h>

#define LOG_LEVEL 3
#include <logging/log.h>
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

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default communication configuration. */
static dwt_config_t config = {5,DWT_PRF_64M,DWT_PLEN_128,DWT_PAC8,9,9,1,DWT_BR_6M8,DWT_PHRMODE_EXT,(129)};

uint8_t this_initiator_node_id  = 1;
uint8_t responder_node_id       = 2;

#define TX_ANT_DLY 16436

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) 
 * conversion factor.
 * 1 uus = 512 / 499.2 usec and 1 usec = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536



#define POLL_TX_TO_RESP_RX_DLY_UUS 300
#define RESP_RX_TO_FINAL_TX_DLY_UUS 4000

#define STACKSIZE 2048

void initiator_thread();
K_THREAD_DEFINE(initiator_main, STACKSIZE, initiator_thread, NULL, NULL, NULL, 99, 0, 0);

void initiator_thread(void)
{
	gpio_pin_init();
	APP_CLEAR;
	APP_SET;
	APP_CLEAR;
    LOG_INF("initiator_thread> starting");

    mp_start(config);


    dwt_setleds(1);
    k_yield();
    uint8_t sequence = 0;
    while (1) {
        //APP_SET_CLEAR 
        // - pulse1: 'tx 1st till rx resp' ; 
        // - pulse2: 'tx final delayed till sent'
        uint32_t reg1 = mp_get_status();
        LOG_INF("initiator> sequence(%u) starting ; statusreg = 0x%08x",sequence,reg1);
        mp_rx_after_tx(POLL_TX_TO_RESP_RX_DLY_UUS);

        msg_header_t twr_poll = {msg_id_t::twr_1_poll, sequence, this_initiator_node_id , responder_node_id,0};
        APP_SET;
        mp_request(twr_poll);

        if(mp_receive(msg_id_t::twr_2_resp))
        {
            APP_CLEAR;
            uint64_t poll_tx_ts = get_tx_timestamp_u64();// DWT_TIME
            uint64_t resp_rx_ts = get_rx_timestamp_u64();// DWT_TIME
            //tx res 9 bits (8 bits shit and 1 bit mask)
            uint32_t final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;   //tranceiver time format
            //prorammed TX + antenna delay
            uint64_t final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;          //host time format

            msg_twr_final_t twr_final;
            twr_final.header = twr_poll.header;//keep same source and dest
            twr_final.header.id = msg_id_t::twr_3_final;
            twr_final.poll_tx_ts = (uint32_t)poll_tx_ts;//trunc 64 bits to 32 bits
            twr_final.resp_rx_ts = (uint32_t)resp_rx_ts;//trunc 64 bits to 32 bits
            twr_final.final_tx_ts = (uint32_t)final_tx_ts;//trunc 64 bits to 32 bits
            APP_SET;
            if(mp_send_at((uint8_t*)&twr_final, sizeof(msg_twr_final_t), final_tx_time))
            {
                APP_CLEAR;
                printk("initiator> success with frame %u\n", sequence);
                LOG_DBG("initiator> poll_tx= 0x%08llx ; resp_rx= 0x%08llx\n", poll_tx_ts, resp_rx_ts);
                LOG_DBG("initiator> final_tx(ant)= 0x%08llx ; final_tx(chip)= 0x%04x\n", final_tx_ts, final_tx_time);
            }else{
                LOG_WRN("mp_send_at(twr_3_final) fail at sequence %u",sequence);
                APP_CLEAR;
            }
        }else{
            LOG_WRN("mp_receive(twr_2_resp) fail at sequence %u",sequence);
            APP_CLEAR;
        }
        uint32_t reg2 = mp_get_status();
        LOG_INF("initiator> sequence(%u) over; statusreg = 0x%08x",sequence,reg2);
        sequence++;
        k_sleep(K_MSEC(RNG_DELAY_MS));
    }
}
