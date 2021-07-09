#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>
#include <usb/usb_device.h>

#include <json.hpp>
#include <simplemesh.h>
#include <meshposition.h>

#include <string>
#include <map>

std::string uid,uwb_cmd;
json j_uwb_cmd,jconfig;
bool do_reconfigure = false;
const uint32_t g_mesh_alive_loop_sec = 20;


#define STACKSIZE 4096
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

void rx_topic_json_handler(std::string &topic, json &data)
{
	if(data.contains("dwt_config")){
		j_uwb_cmd = data["dwt_config"];
		uwb_cmd = "dwt_config";
		k_sem_give(&sem_uwb_cmd);//giving sem to higher prio
	}
	else if(data.contains("twr_command")){
		j_uwb_cmd = data["twr_command"];
		uwb_cmd = "twr_command";
		k_sem_give(&sem_uwb_cmd);
	}
}

void mesh_start()
{
	std::string uid = sm_get_uid();
	printk("sm>DWM1001-Dev nRF-UID (%s)\n",uid.c_str());

	sm_set_callback_rx_json(rx_topic_json_handler);
	sm_start();

	json j_mesh_state;
	j_mesh_state["simplemesh"] = "started";
	mesh_bcast_json(j_mesh_state);
	printf("sm> %s\n",j_mesh_state.dump().c_str());
}

void uwb_thread(void)
{
	app_gpio_init();
	mesh_start();//start mesh first to get short id

	static dwt_config_t config = {5,DWT_PRF_64M,DWT_PLEN_128,DWT_PAC8,9,9,1,DWT_BR_6M8,DWT_PHRMODE_EXT,(129)};
	mp_start(config);
	mp_conf_to_json(config,jconfig);
	mesh_bcast_json(jconfig);
	printf("dwt_config>%s\n",jconfig.dump().c_str());

	int cmd_count = 0;
    while (1) {
		if(k_sem_take(&sem_uwb_cmd,K_MSEC(100)) == 0){
			if(uwb_cmd.compare("dwt_config") == 0){
				for (auto& [key, value] : j_uwb_cmd.items()) {
					jconfig[key] = value;
				}
				mp_json_to_conf(jconfig,config);
				dwt_configure(&config);
				printf("new dwt config :\n%s\n",jconfig.dump(2).c_str());
				mesh_bcast_json(jconfig);
			}
			else if(uwb_cmd.compare("twr_command") == 0){
				uint8_t initiator = j_uwb_cmd["initiator"];
				uint8_t responder = j_uwb_cmd["responder"];
				uint64_t rx_delta_ms = 0;
				if(j_uwb_cmd.contains("at_ms")){
					rx_delta_ms = j_uwb_cmd["at_ms"];
				}
				uint8_t this_node_id = sm_get_sid();
				//responder starts first
				if(responder == this_node_id){
					sm_rx_delay_ms(rx_delta_ms);
					twr_respond(cmd_count,initiator,responder,j_uwb_cmd);
					mesh_bcast_json(j_uwb_cmd);
				}else if(initiator == this_node_id){
					sm_rx_delay_ms(rx_delta_ms);
					twr_intiate(cmd_count,initiator,responder);
				}
				printf("uwb>twr_command done (%u)->(%u)\n",initiator,responder);
			}
			cmd_count++;
		}
    }
}
