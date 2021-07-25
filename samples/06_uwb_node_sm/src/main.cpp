#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>
#include <usb/usb_device.h>
#include <sys/reboot.h>

#include <json.hpp>
#include <simplemesh.h>
#include <meshposition.h>

#include <string>
#include <vector>

std::string uid,uwb_cmd;
json j_uwb_cmd,jconfig,jresponse,j_sw;
uint8_t source_uwb_cmd;
bool do_reconfigure = false;
const uint32_t g_mesh_alive_loop_sec = 20;
extern bool critical_parse;

#define STACKSIZE 8192
LOG_MODULE_REGISTER(uwb_main, LOG_LEVEL_DBG);

void uwb_thread();
K_SEM_DEFINE(sem_uwb_cmd, 0, 1);
K_THREAD_DEFINE(uwb_main, STACKSIZE, uwb_thread, NULL, NULL, NULL, 10, 0, 0);

#if (CONFIG_MP_GPIO_DEBUG || CONFIG_SM_GPIO_DEBUG)
	#include <drivers/gpio.h>
	const struct device *gpio_dev;

	#if CONFIG_MP_GPIO_DEBUG
		//0.625 us per toggle
		#define PIN_MP_SET 		gpio_pin_set(gpio_dev, CONFIG_MP_PIN_APP, 1)
		#define PIN_MP_CLEAR 	gpio_pin_set(gpio_dev, CONFIG_MP_PIN_APP, 0)
	#else
		#define PIN_MP_SET
		#define PIN_MP_CLEAR
	#endif

	#if CONFIG_SM_GPIO_DEBUG
		//0.625 us per toggle
		#define PIN_SM_SET 		gpio_pin_set(gpio_dev, CONFIG_SM_PIN_APP, 1)
		#define PIN_SM_CLEAR 	gpio_pin_set(gpio_dev, CONFIG_SM_PIN_APP, 0)
	#else
		#define PIN_SM_SET
		#define PIN_SM_CLEAR
	#endif
#endif

void app_gpio_init()
{
	#if (CONFIG_MP_GPIO_DEBUG || CONFIG_SM_GPIO_DEBUG)
		gpio_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
		#if CONFIG_MP_GPIO_DEBUG
			gpio_pin_configure(gpio_dev, CONFIG_MP_PIN_APP, GPIO_OUTPUT_ACTIVE);
			twr_gpio_init(gpio_dev);
			PIN_MP_CLEAR;
			PIN_MP_SET;
			PIN_MP_CLEAR;
		#endif
		#if CONFIG_SM_GPIO_DEBUG
			gpio_pin_configure(gpio_dev, CONFIG_SM_PIN_APP, GPIO_OUTPUT_ACTIVE);
			sm_gpio_init(gpio_dev);
			PIN_SM_CLEAR;
			PIN_SM_SET;
			PIN_SM_CLEAR;
		#endif
	#endif
}

void report_sw_version()
{
	mesh_bcast_json(j_sw);
	printf("sw>%s\n",j_sw.dump().c_str());
}

void rx_topic_json_handler(uint8_t source, std::string &topic, json &data)
{
	source_uwb_cmd = source;
	if(data.contains("uwb_cmd")){
		uwb_cmd = data["uwb_cmd"];
		j_uwb_cmd = data;
		k_sem_give(&sem_uwb_cmd);//giving sem to higher prio
	}else if(data.contains("rf_cmd")){
		if(is_self(topic)){
			sm_diag(data);//giving sem to higher prio
		}
	}else if(data.contains("sys_cmd")){
		if(is_self(topic)){
			if(data["sys_cmd"] == "reboot"){
				sys_reboot(SYS_REBOOT_WARM);//param unused on ARM-M
			}else if(data["sys_cmd"] == "sw_version"){
				report_sw_version();
			}
		}
	}
}

void mesh_start()
{
	std::string uid = sm_get_uid();
	printf("sm>DWM1001-Dev nRF-UID (%s)\n",uid.c_str());

	sm_start();//assigns uid, sid
	sm_set_callback_rx_json(rx_topic_json_handler);

	j_sw["date"] = std::string(__DATE__);
	j_sw["time"] = std::string(__TIME__);
	j_sw["commit"] = std::string(CONFIG_COMMIT_VAR);
	report_sw_version();

	printf("sm> started\n");
}

uint64_t get_default(json &data,const char*field,uint64_t default_val)
{
	if(data.contains(field)){
		return data[field];
	}else{
		return default_val;
	}
}

void command_twr(json &data,int sequence)
{
	if(data.contains("initiator")){
		data["initiators"].push_back(data["initiator"]);
	}
	if(data.contains("responder")){
		data["responders"].push_back(data["responder"]);
	}
	uint64_t rx_delta_ms 	= get_default(data,"at_ms",0);
	uint64_t count 			= get_default(data,"count",1);
	uint64_t count_ms 		= get_default(data,"count_ms",0);
	uint64_t step_ms 		= get_default(data,"step_ms",0);
	uint8_t this_node_id	= sm_get_sid();
	sm_stop_rx();
	int64_t start = sm_rx_sync_ms(sequence,rx_delta_ms);
	int lseq = 0;//local sequence for the whole request including all counts
	for(uint64_t i=0;i<count;i++){
		if((i!=0) && (count_ms!=0)){
			start = sm_sync_ms(lseq,start,count_ms);
		}
		uint64_t j = 0;
		int64_t step_time = start;
		for(const auto& initiator: data["initiators"]){
			for(const auto& responder: data["responders"]){
				if((j!=0) && (step_ms!=0)){
					step_time = sm_sync_ms(lseq,step_time,step_ms);
				}
				if(responder == this_node_id){
					twr_respond(lseq,initiator,responder,jresponse);
					jresponse["uwb_cmd"] = "twr";
					jresponse["initiator"] = initiator;
					jresponse["responder"] = responder;
					jresponse["seq"] = lseq;
					mesh_bcast_json(jresponse);
					jresponse.clear();
					k_sleep(K_MSEC(1));//to handle Tx by RF thread
				}else if(initiator == this_node_id){
					twr_intiate(lseq,initiator,responder);
				}
				j++;
				lseq++;
			}
		}
	}
	sm_start_rx();
	printf("uwb>twr_command done\n");
}

void command_ping(json &data,int sequence)
{
	uint8_t pinger = data["pinger"];
	uint8_t target = data["target"];
	uint64_t rx_delta_ms = 0;
	if(data.contains("at_ms")){
		rx_delta_ms = data["at_ms"];
	}
	uint64_t count = 1;
	if(data.contains("count")){
		count = data["count"];
	}
	uint64_t count_ms = 0;
	if(data.contains("count_ms")){
		count_ms = data["count_ms"];
	}
	int lseq = 0;
	uint8_t this_node_id = sm_get_sid();
	sm_stop_rx();
	if(target == this_node_id){
		int64_t start = sm_rx_sync_ms(lseq,rx_delta_ms);
		for(uint64_t i=0;i<count;i++){
			if((i!=0) && (count_ms!=0)){
				start = sm_sync_ms(lseq,start,count_ms);
			}
			uwb_ping_rx(sequence,pinger,target,jresponse);
			jresponse["uwb_cmd"] = "ping";
			mesh_bcast_json(jresponse);
			jresponse.clear();
			k_sleep(K_MSEC(1));//to handle Tx by RF thread
			lseq++;
		}
	}else if(pinger == this_node_id){
		int64_t start = sm_rx_sync_ms(lseq,rx_delta_ms);
		for(uint64_t i=0;i<count;i++){
			if((i!=0) && (count_ms!=0)){
				start = sm_sync_ms(lseq,start,count_ms);
			}
			uwb_ping(sequence,pinger,target);
			lseq++;
		}
	}
	sm_start_rx();
	printf("uwb>twr_command done (%u)->(%u)\n",pinger,target);
}

void uwb_thread(void)
{
	app_gpio_init();
	mesh_start();//start mesh first to get short id

	static dwt_config_t config = {5,DWT_PRF_64M,DWT_PLEN_128,DWT_PAC8,9,9,1,DWT_BR_6M8,DWT_PHRMODE_EXT,(129)};
	mp_start(config);
	mp_conf_to_json(config,jconfig);
	printf("dwt_config>%s\n",jconfig.dump(2).c_str());

	int cmd_count = 0;
    while (1) {
		if(k_sem_take(&sem_uwb_cmd,K_MSEC(100)) == 0){
			if(uwb_cmd.compare("config") == 0){
				for (auto& [key, value] : j_uwb_cmd.items()) {
					if(jconfig.contains(key)){
						jconfig[key] = value;
					}
				}
				mp_json_to_conf(jconfig,config);
				dwt_configure(&config);
				printf("new dwt config :\n%s\n",jconfig.dump(2).c_str());
				mesh_bcast_json(jconfig);
			}
			else if(uwb_cmd.compare("twr") == 0){
				command_twr(j_uwb_cmd,cmd_count);
			}else if(uwb_cmd.compare("ping") == 0){
				command_ping(j_uwb_cmd,cmd_count);
			}else if(uwb_cmd.compare("cir_acc") == 0){
				printf("before cir_acc()\n");
				uwb_cir_acc(source_uwb_cmd);//send acc to source read one more : DW1000_Software_API_Guide
			}
			cmd_count++;
		}
		if(critical_parse){
			sys_reboot(SYS_REBOOT_WARM);//param unused on ARM-M
		}
    }
}
