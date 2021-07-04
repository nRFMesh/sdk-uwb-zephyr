
#include <zephyr.h>
#include <meshposition.h>
#include <string>
#include <map>
#include <list>

#include <logging/log.h>
#include <stdio.h>

#include <drivers/gpio.h>

LOG_MODULE_REGISTER(mesh_twr, LOG_LEVEL_ERR);

#ifdef CONFIG_MP_GPIO_DEBUG
	#include <drivers/gpio.h>

	const struct device *twr_gpio_dev;
	//0.625 us per toggle
	#define APP_SET 	gpio_pin_set(twr_gpio_dev, CONFIG_MP_PIN_APP, 1)
	#define APP_CLEAR 	gpio_pin_set(twr_gpio_dev, CONFIG_MP_PIN_APP, 0)

	void twr_gpio_init(const struct device *gpio_dev)
	{
		twr_gpio_dev = gpio_dev;
	}
#else
	#define APP_SET 	
	#define APP_CLEAR 	
#endif

//-------- responder params --------
/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion 
 * factor.
 * 1 uus = 512 / 499.2 usec and 1 usec = 499.2 * 128 dtu.
 */
#define UUS_TO_DWT_TIME 65536

#define POLL_RX_TO_RESP_TX_DLY_UUS 1000
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

//-------- intiator params --------
#define TX_ANT_DLY 16436
/*rx twr_2_resp after tx twr_1_poll
 protected by responder's mp_request_at(twr_2_resp):POLL_RX_TO_RESP_TX_DLY_UUS
*/
#define POLL_TX_TO_RESP_RX_DLY_UUS 300

#define RESP_RX_TO_FINAL_TX_DLY_UUS 1000

#define MAX_RX_TIMEOUT 65535

//starting this sequence waits for max 65 ms
void twr_respond(uint8_t sequence,uint8_t source_initiator,uint8_t dest_responder)
{
	//APP_SET_CLEAR 
	// - pulse1: '1st rx 	: entrance -> receive pending done' ; 
	// - pulse2: 'request_at: start request resp_2 tx till sent' ; 
	// - pulse3: 'final rx	: pending for receive final_3'
	// - pulse4: 'computing : distance'
	APP_SET;
	uint32_t reg1 = mp_get_status();
	LOG_INF("responder> sequence(%u) starting ; statusreg = 0x%08x",sequence,reg1);
	mp_rx_now(MAX_RX_TIMEOUT);
	msg_header_t rx_poll_msg;
	if(mp_receive(msg_id_t::twr_1_poll,rx_poll_msg)){
		APP_CLEAR;
		if(rx_poll_msg.header.dest != dest_responder){
			LOG_ERR("not for this node poll responder dest(%d) expected dest(%d)",rx_poll_msg.header.dest,dest_responder);
			return;
		}
		if(rx_poll_msg.header.source != source_initiator){
			LOG_ERR("not for this node poll responder dest(%d) expected dest(%d)",rx_poll_msg.header.dest,dest_responder);
			return;
		}
		uint64_t poll_rx_ts = get_rx_timestamp_u64();

		mp_rx_after_tx(RESP_TX_TO_FINAL_RX_DLY_UUS);
		
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
				printf("%s", dist_str);
			}else{
				LOG_WRN("mp_receive(twr_3_final) fail at rx frame %u",rx_poll_msg.header.sequence);
				APP_CLEAR;
			}
		}else{
			APP_CLEAR;
			LOG_WRN("mp_request_at(twr_2_resp) fail at rx frame %u",rx_poll_msg.header.sequence);
		}
	}else{
		APP_CLEAR;
	}

	uint32_t reg2 = mp_get_status();
	LOG_INF("responder> sequence(%u) over; statusreg = 0x%08x",sequence,reg2);
}

void twr_intiate(uint8_t sequence,uint8_t source_initiator,uint8_t dest_responder)
{
	//APP_SET_CLEAR 
	// - pulse1: 'entrance : reg read and request start tx no-wait' ; 
	// - pulse2: 'request-receive : tx 1st till rx resp' ; 
	// - pulse3: 'send_at : tx final delayed till sent'
	APP_SET;
	uint32_t reg1 = mp_get_status();
	LOG_INF("initiator> sequence(%u) starting ; statusreg = 0x%08x",sequence,reg1);
	mp_rx_after_tx(POLL_TX_TO_RESP_RX_DLY_UUS);

	msg_header_t twr_poll = {msg_id_t::twr_1_poll, sequence, source_initiator , dest_responder,0};
	mp_request(twr_poll);
	APP_CLEAR;
	APP_SET;
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
			printf("initiator> success with frame %u\n", sequence);
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
}
