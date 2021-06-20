#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>
#include <usb/usb_device.h>
#include <simplemesh.h>
#include <json.hpp>
#include <string>
#include <map>
#include <list>

#include <drivers/dw1000/deca_device_api.h>
#include <drivers/dw1000/deca_regs.h>
#include <drivers/dw1000/deca_spi.h>
#include <drivers/dw1000/port.h>

std::string uid;
std::string full_topic;
json j_mesh_state, j_uwb_state,jconfig;
bool do_reconfigure = false;

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

std::map<std::string,uint8_t> invmap_dataRate,invmap_rxPAC,invmap_txPreambLength;

#define STACKSIZE 2048
LOG_MODULE_REGISTER(listener_main, LOG_LEVEL_DBG);

void listener_thread();
K_THREAD_DEFINE(listener_main, STACKSIZE, listener_thread, NULL, NULL, NULL, 99, 0, 0);

void mesh_thread();
K_THREAD_DEFINE(mesh_parser, STACKSIZE, mesh_thread, NULL, NULL, NULL, 80, 0, 0);

void inverse_map(std::map<uint8_t,std::string> &map,std::map<std::string,uint8_t> &invmap)
{
	for (auto& [key, value] : map) {
		invmap.insert(std::make_pair(value,key));
	}
}

void rx_topic_json_handler(std::string &topic, json &data)
{
	printk("topic = %s ; json :\n",topic.c_str());
	printk("%s\n",data.dump(2).c_str());
	if(data.contains("dwt_config")){
		for (auto& [key, value] : data["dwt_config"].items()) {
			jconfig[key] = value;
		}
		do_reconfigure = true;
	}
}

void mesh_thread()
{
	#ifdef CONFIG_USB
		int ret;
		ret = usb_enable(NULL);
		if (ret != 0) {
			LOG_ERR("Failed to enable USB");
			return;
		}
	#endif

	uid = sm_get_uid();
	full_topic = sm_get_topic();
	printk("mesh>Hello Simple Mesh from DWM1001 UID (%s)\n",uid.c_str());
	printk("broadcasting on self topic (%s)\n",full_topic.c_str());

	sm_set_callback_rx_json(rx_topic_json_handler);
	sm_start();

	int loop = 0;
	while (1) {
		j_mesh_state["mesh_loop"] = loop;
		mesh_bcast_json(j_mesh_state);
		printf("mesh> %s\n",j_mesh_state.dump().c_str());
		k_sleep(K_SECONDS(10));
		loop++;
	}
}


static uint16 frame_len = 0;
static uint32 status_reg = 0;
#define FRAME_LEN_MAX 127
static uint8 rx_buffer[FRAME_LEN_MAX];
static dwt_config_t config = {5,DWT_PRF_64M,DWT_PLEN_128,DWT_PAC8,9,9,1,DWT_BR_6M8,DWT_PHRMODE_EXT,(129)};

void print_time_buffer(uint8 *rx_buffer, uint16 len)
{
	int time_stamp = k_uptime_get();
	printf("%d> ",time_stamp);
	for (int i = 0 ; i < frame_len; i++ )
	{
		printf("%02x ",rx_buffer[i]);
	}
	printf("\r\n");
}

json uwb_status_to_json(uint32 status_reg)
{
	std::list<std::string> flags;
	for (auto& [key, value] : map_reg_status) {
		if(status_reg & key)flags.push_back(value);
	}
	return json(flags);
}

void sniff_ms(int time)
{
    int start = k_uptime_get();
    int finish = start + time;
    printf("sniff> start(%d) -> finish(%d)\n",start,finish);

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
				print_time_buffer(rx_buffer, frame_len);
            }
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
        }
        else {
			json jstat = uwb_status_to_json(status_reg);
            printf("sniff> 0x%04lX :\n%s\n",status_reg,jstat.dump(2).c_str());
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
    }
}

template <class Tfirst, class Tsec>
Tsec set_from_map(Tfirst &key,std::map<Tfirst,Tsec> &map,Tsec &value)
{
	if(map.find(key)!=map.end()){
		return map[key];
	}else{
		return value;
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

uint8_t set_from_map_uint(std::string &key,std::map<std::string,uint8_t> &map)
{
	if(map.find(key)!=map.end()){
		return map[key];
	}else{
		return 0;
	}
}

void uwb_struct_to_json(dwt_config_t &conf,json &jconf)
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

void uwb_json_to_struct(json &jconf,dwt_config_t &conf)
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

void listener_thread(void)
{
    openspi();
    port_set_dw1000_slowrate();

    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR) {
        LOG_ERR("dwt_initialise failed");
        return;
    }
    port_set_dw1000_fastrate();

    dwt_configure(&config);
	inverse_map(map_txPreambLength,invmap_txPreambLength);
	inverse_map(map_rxPAC,invmap_rxPAC);
	inverse_map(map_txPreambLength,invmap_dataRate);
	uwb_struct_to_json(config,jconfig);
    printf("uwb config :\n%s\n",jconfig.dump(2).c_str());

    dwt_setleds(1);
	int loop = 0;
    while (1) {
		if(do_reconfigure){
			uwb_json_to_struct(jconfig,config);
			dwt_configure(&config);
		    printf("new dwt config :\n%s\n",jconfig.dump(2).c_str());
			mesh_bcast_json(jconfig);
			do_reconfigure = false;
		}
		j_uwb_state["uwb_loop"] = loop;
		mesh_bcast_json(j_uwb_state);
		printf("listener> %s\n",j_uwb_state.dump().c_str());
		k_sleep(K_MSEC(100));
		loop++;
        sniff_ms(k_ms_to_ticks_floor32(5000));
    }
}
