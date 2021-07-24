
#include <zephyr.h>
#include <meshposition.h>
#include <string>
#include <map>
#include <list>

#include <logging/log.h>
#include <sys/printk.h>

#include <drivers/dw1000/deca_regs.h>
#include <drivers/dw1000/deca_spi.h>
#include <drivers/dw1000/port.h>

LOG_MODULE_REGISTER(meshposition, LOG_LEVEL_ERR);

#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

static uint32_t status_reg = 0;


std::map<std::string,uint8_t> invmap_dataRate,invmap_rxPAC,invmap_txPreambLength;

std::map<unsigned long,std::string> map_reg_status {
	{SYS_STATUS_IRQS,	   	"Interrupt Request Status READ ONLY"},
	{SYS_STATUS_CPLOCK,	   	"Clock PLL Lock"},
	{SYS_STATUS_ESYNCR,	   	"External Sync Clock Reset"},
	{SYS_STATUS_AAT,	   	"Automatic Acknowledge Trigger"},
	{SYS_STATUS_TXFRB,	   	"Transmit Frame Begins"},
	{SYS_STATUS_TXPRS,	   	"Transmit Preamble Sent"},
	{SYS_STATUS_TXPHS,	   	"Transmit PHY Header Sent"},
	{SYS_STATUS_TXFRS,	   	"Transmit Frame Sent"},
	{SYS_STATUS_RXPRD,	   	"Receiver Preamble Detected status"},
	{SYS_STATUS_RXSFDD,	   	"Receiver Start Frame Delimiter Detected."},
	{SYS_STATUS_LDEDONE,   	"LDE processing done"},
	{SYS_STATUS_RXPHD,	   	"Receiver PHY Header Detect"},
	{SYS_STATUS_RXPHE,	   	"Receiver PHY Header Error"},
	{SYS_STATUS_RXDFR,	   	"Receiver Data Frame Ready"},
	{SYS_STATUS_RXFCG,	   	"Receiver FCS Good"},
	{SYS_STATUS_RXFCE,	   	"Receiver FCS Error"},
	{SYS_STATUS_RXRFSL,	   	"Receiver Reed Solomon Frame Sync Loss"},
	{SYS_STATUS_RXRFTO,	   	"Receive Frame Wait Timeout"},
	{SYS_STATUS_LDEERR,	   	"Leading edge detection processing error"},
	{SYS_STATUS_reserved,  	"bit19 reserved"},
	{SYS_STATUS_RXOVRR,	   	"Receiver Overrun"},
	{SYS_STATUS_RXPTO,	   	"Preamble detection timeout"},
	{SYS_STATUS_GPIOIRQ,	"GPIO interrupt"},
	{SYS_STATUS_SLP2INIT,	"SLEEP to INIT"},
	{SYS_STATUS_RFPLL_LL,	"RF PLL Losing Lock"},
	{SYS_STATUS_CLKPLL_LL,	"Clock PLL Losing Lock"},
	{SYS_STATUS_RXSFDTO,	"Receive SFD timeout"},
	{SYS_STATUS_HPDWARN,	"Half Period Delay Warning"},
	{SYS_STATUS_TXBERR,	   	"Transmit Buffer Error"},
	{SYS_STATUS_AFFREJ,	   	"Automatic Frame Filtering rejection"},
	{SYS_STATUS_HSRBP,	   	"Host Side Receive Buffer Pointer"},
	{SYS_STATUS_ICRBP,	   	"IC side Receive Buffer Pointer READ ONLY"},
	{SYS_STATUS_RXRSCS,		"Receiver Reed-Solomon Correction Status"},
	{SYS_STATUS_RXPREJ,		"Receiver Preamble Rejection"},
	{SYS_STATUS_TXPUTE,		"Transmit power up time error"},
	{SYS_STATUS_TXERR,       "Transmit Error TXPUTE or HPDWARN"},
	{SYS_STATUS_ALL_RX_GOOD, "Any RX events"},
	{SYS_STATUS_ALL_DBLBUFF, "Any double buffer events"},
	{SYS_STATUS_ALL_RX_ERR,  "Any RX errors"},
	{SYS_STATUS_ALL_RX_TO,   "User defined RX timeouts"},
	{SYS_STATUS_ALL_TX,      "Any TX events"}
	};

std::map<uint8_t,std::string> map_txPreambLength {
	{0x0C,"DWT_PLEN_4096"},
	{0x28,"DWT_PLEN_2048"},
	{0x18,"DWT_PLEN_1536"},
	{0x08,"DWT_PLEN_1024"},
	{0x34,"DWT_PLEN_512"},
	{0x24,"DWT_PLEN_256"},
	{0x14,"DWT_PLEN_128"},
	{0x04,"DWT_PLEN_64"}
	};

std::map<uint8_t,std::string> map_rxPAC {
	{0,"DWT_PAC8"},
	{1,"DWT_PAC16"},
	{2,"DWT_PAC32"},
	{3,"DWT_PAC64"}
	};

std::map<uint8_t,std::string> map_dataRate {
	{0,"DWT_BR_110K"},
	{1,"DWT_BR_850K"},
	{2,"DWT_BR_6M8"}
	};

void inverse_map(std::map<uint8_t,std::string> &map,std::map<std::string,uint8_t> &invmap)
{
	for (auto& [key, value] : map) {
		invmap.insert(std::make_pair(value,key));
	}
}

std::string set_from_map_str(uint8_t &key,std::map<uint8_t,std::string> &map)
{
	if(map.find(key)!=map.end()){
		return map[key];
	}else{
		return "undefined";
	}
}

json mp_status_to_json(uint32_t status_reg)
{
	std::list<std::string> flags;
	for (auto& [key, value] : map_reg_status) {
		if(status_reg & key)flags.push_back(value);
	}
	return json(flags);
}

void mp_status_print(uint32_t status_reg)
{
	for (auto& [key, value] : map_reg_status) {
		if(status_reg & key)
        {
            printk("%s; ",value.c_str());
        }
	}
	printk("\n");
}

void mp_conf_to_json(dwt_config_t &conf,json &jconf)
{
	jconf["chan"] 		= conf.chan;
	jconf["prf"] 		= (conf.prf == DWT_PRF_16M)?"DWT_PRF_16M":"DWT_PRF_64M";
	jconf["txPreambLength"] = set_from_map_str(conf.txPreambLength,map_txPreambLength);
	jconf["rxPAC"] 		= set_from_map_str(conf.rxPAC,map_rxPAC);
	jconf["txCode"] 	= conf.txCode;
	jconf["rxCode"] 	= conf.rxCode;
	jconf["nsSFD"] 		= (conf.nsSFD)?"NonStandard":"Standard";
	jconf["dataRate"] 	= set_from_map_str(conf.dataRate,map_dataRate);
	jconf["phrMode"] 	= (conf.phrMode == DWT_PHRMODE_STD)?"DWT_PHRMODE_STD":"DWT_PHRMODE_EXT";
	jconf["sfdTO"] 		= conf.sfdTO;
}

void mp_json_to_conf(json &jconf,dwt_config_t &conf)
{
	conf.chan				= (uint8) jconf["chan"];
	conf.prf 				= (uint8) (jconf["prf"] == "DWT_PRF_16M")?DWT_PRF_16M:DWT_PRF_64M;
	conf.txPreambLength		= (uint8) invmap_txPreambLength[jconf["txPreambLength"]];
	conf.rxPAC	 			= (uint8) invmap_rxPAC[jconf["rxPAC"]];
	conf.txCode 			= (uint8) jconf["txCode"];
	conf.rxCode 			= (uint8) jconf["rxCode"];
	conf.nsSFD				= (uint8) (jconf["nsSFD"]=="NonStandard")?true:false;
	conf.dataRate			= (uint8) invmap_dataRate[jconf["dataRate"]];
	conf.phrMode			= (uint8) (jconf["phrMode"] == "DWT_PHRMODE_STD")?DWT_PHRMODE_STD:DWT_PHRMODE_EXT;
	conf.sfdTO 	  			= (uint8) jconf["sfdTO"];
}

void mp_start(dwt_config_t &config)
{
    openspi();
    k_sleep(K_MSEC(2));
    port_set_dw1000_slowrate();

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        LOG_WRN("dwt_initialise failed");
        return;
    }
    port_set_dw1000_fastrate();

    dwt_configure(&config);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

	inverse_map(map_txPreambLength,invmap_txPreambLength);
	inverse_map(map_rxPAC,invmap_rxPAC);
	inverse_map(map_txPreambLength,invmap_dataRate);

    dwt_setleds(0);
}

/* RX can be started
 - Immediatley					mp_rx_now()
 - After TX with a given delay 	mp_rx_after_tx()
 - At a given timestamp			not api provided. uses dwt_setdelayedtrxtime() and dwt_rxenable() with DWT_START_RX_DELAYED
*/
void mp_rx_now(uint16_t timeout)
{
	dwt_setrxtimeout(timeout); // 0 : disable timeout
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void mp_rx_after_tx(uint32_t delay_us,uint16_t timeout)
{
    dwt_setrxaftertxdelay(delay_us);
    dwt_setrxtimeout(timeout);
}

uint32_t mp_poll_rx()
{
	uint32_t l_status_reg;
	while (!((l_status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
	{
	};
	return l_status_reg;
}

uint32_t mp_get_status()
{
	return dwt_read32bitreg(SYS_STATUS_ID);
}

//TODO check that we're on listening state
bool mp_receive(uint8_t* data, uint16_t expected_size)
{
	bool result = false;
	status_reg = mp_poll_rx();
	if(status_reg & SYS_STATUS_RXFCG){
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);//clearing any rx and tx
		uint16_t size = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
		if (size == expected_size) {
			dwt_readrxdata(data, size, 0);
			result = true;
		}else{
			LOG_ERR("rx size %u != expected_size %u",size,expected_size);
		}
	}else{
		dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);//clear errors
		dwt_rxreset();
		LOG_WRN("mp_receive() no 'RX Frame Checksum Good'");
		uint32_t reg2 = mp_get_status();
		uint32_t reg_removed = (status_reg ^ reg2);
		LOG_WRN("mp_receive() reg1 = 0x%08x ; reg2 = 0x%08x ; removed =  0x%08x",status_reg,reg2,reg_removed);
		mp_status_print(reg_removed);
		status_reg = reg2;
	}

	return result;
}

bool mp_receive(msg_id_t id)
{
	bool result = false;
	msg_header_t message;
	if(mp_receive((uint8_t*)&message, sizeof(msg_header_t))){
		if(message.header.id == id){//TODO check that dest is self when id is moved as global here
			result = true;
		}else{
			LOG_ERR("mp_receive() id mismatch id(%u/%u)",(uint8_t)id,(uint8_t)message.header.id);
		}
	}else{
		LOG_ERR("mp_receive(%u) fail",(uint8_t)id);
	}
	return result;
}

bool mp_receive(msg_id_t id,msg_header_t& message)
{
	bool result = false;
	if(mp_receive((uint8_t*)&message, sizeof(msg_header_t))){
		if(message.header.id == id){//TODO check that dest is self when id is moved as global here
			result = true;
		}else{
			LOG_ERR("mp_receive() mismatch id(%u/%u)",(uint8_t)id,(uint8_t)message.header.id);
		}
	}else{
		LOG_ERR("mp_receive(%u,header) fail",(uint8_t)id);
	}
	return result;
}

bool mp_receive(msg_id_t id,msg_twr_final_t& final_msg)
{
	bool result = false;
	if(mp_receive((uint8_t*)&final_msg, sizeof(msg_twr_final_t))){
		if(final_msg.header.id == id){//TODO check that dest is self when id is moved as global here
			result = true;
		}else{
			LOG_ERR("mp_receive() id mismatch(%u/%u)",(uint8_t)id,(uint8_t)final_msg.header.id);
		}
	}else{
		LOG_ERR("mp_receive(%u,final_msg) fail",(uint8_t)id);
	}
	return result;
}


//a request expects a response
void mp_request(uint8_t* data, uint16_t size)
{
	dwt_writetxdata(size, data, 0);//0 offset
	dwt_writetxfctrl(size, 0, 1);//ranging bit unused by DW1000
	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);//switch to rx after `setrxaftertxdelay`
}

void mp_request(msg_header_t &message)
{
	mp_request((uint8_t*)&message,(uint16_t)sizeof(msg_header_t));
}

//no response expected
void mp_send(uint8_t* data, uint16_t size)
{
	dwt_writetxdata(size, data, 0); 
	dwt_writetxfctrl(size, 0, 1); 
	dwt_starttx(DWT_START_TX_IMMEDIATE);
}

void recover_tx_errors()
{
	uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);
	if(status & SYS_STATUS_TXERR)
	{
		LOG_INF("recovering TX errors 0x%08x", (status & SYS_STATUS_TXERR));
		dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8)SYS_CTRL_TRXOFF);
	}
}

/*! @fn mp_send_at()
 * @param tx_time : higher 31 bits (bit 0 ignored) from system time which is 40 bits
 * unit is ~ 8 ns or (1/124,8) us ~= 0.00801282... us
 * all _at() functions :
 * - The Frame RMARKER
 * - do not include antenna delay
 * - do not consider preamble time (starts earlier)
*/
bool mp_send_at(uint8_t* data, uint16_t size, uint64_t tx_time, uint8_t flag)
{
	uint32_t reg1 = dwt_read32bitreg(SYS_STATUS_ID);
	//next tansmission timestamp (although same reg can be used for delayed reception)
	dwt_setdelayedtrxtime(tx_time);
	dwt_writetxdata(size, data, 0); 
	dwt_writetxfctrl(size, 0, 1); 
	bool late = dwt_starttx(DWT_START_TX_DELAYED | flag);
	if(late == DWT_SUCCESS){
		while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
		{
		};
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
		return true;
	}else{
		uint32_t reg2 = dwt_read32bitreg(SYS_STATUS_ID);
		recover_tx_errors();
		status_reg = dwt_read32bitreg(SYS_STATUS_ID);
		LOG_WRN("mp_send_at() - dwt_starttx() late");
		LOG_WRN("reg1 = 0x%08x ; reg2 = 0x%08x; regnow = 0x%08x",reg1,reg2,status_reg);
		return false;
	}
}

bool mp_request_at(uint8_t* data, uint16_t size, uint64_t tx_time)
{
	return mp_send_at(data,size,tx_time,DWT_RESPONSE_EXPECTED);
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
uint64_t get_tx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
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
uint64_t get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int i;

    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

