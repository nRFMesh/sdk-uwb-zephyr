#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>
#include <usb/usb_device.h>

#include <json.hpp>
#include <simplemesh.h>
#include <meshposition.h>

#include <string>
#include <map>

#include <drivers/dw1000/deca_device_api.h>
#include <drivers/dw1000/deca_regs.h>

std::string uid;
std::string full_topic;
json j_mesh_state, j_uwb_state,jconfig;
bool do_reconfigure = false;

#define STACKSIZE 2048
LOG_MODULE_REGISTER(listener_main, LOG_LEVEL_DBG);

void listener_thread();
K_THREAD_DEFINE(listener_main, STACKSIZE, listener_thread, NULL, NULL, NULL, 99, 0, 0);

void mesh_thread();
K_THREAD_DEFINE(mesh_parser, STACKSIZE, mesh_thread, NULL, NULL, NULL, 80, 0, 0);


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

void sniff_ms(int time)
{
    int start = k_uptime_get();
    int finish = start + time;
    printf("sniff> start(%d) -> finish(%d)\n",start,finish);

    while (k_uptime_get() < finish) {
		memset(rx_buffer,0,FRAME_LEN_MAX);
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
			json jstat = mp_status_to_json(status_reg);
            printf("sniff> 0x%04lX :\n%s\n",status_reg,jstat.dump(2).c_str());
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
    }
}

void listener_thread(void)
{
	static dwt_config_t config = {5,DWT_PRF_64M,DWT_PLEN_128,DWT_PAC8,9,9,1,DWT_BR_6M8,DWT_PHRMODE_EXT,(129)};

	mp_start(config);

	mp_conf_to_json(config,jconfig);
    printf("uwb config :\n%s\n",jconfig.dump(2).c_str());

	int loop = 0;
    while (1) {
		if(do_reconfigure){
			mp_json_to_conf(jconfig,config);
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
