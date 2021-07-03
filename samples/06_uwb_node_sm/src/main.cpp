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

std::string uid,uwb_cmd;
json j_uwb_cmd,jconfig;
bool do_reconfigure = false;
const uint32_t g_mesh_alive_loop_sec = 60;


#define STACKSIZE 2048
LOG_MODULE_REGISTER(uwb_main, LOG_LEVEL_DBG);

void uwb_thread();
K_SEM_DEFINE(sem_uwb_cmd, 0, 1);
K_THREAD_DEFINE(uwb_main, STACKSIZE, uwb_thread, NULL, NULL, NULL, 99, 0, 0);

void mesh_thread();
K_THREAD_DEFINE(mesh_parser, STACKSIZE, mesh_thread, NULL, NULL, NULL, 80, 0, 0);

void rx_topic_json_handler(std::string &topic, json &data)
{
	if(data.contains("dwt_config")){
		j_uwb_cmd = data["dwt_config"];
		uwb_cmd = "dwt_config";
		k_sem_give(&sem_uwb_cmd);
	}
}

void mesh_thread()
{
	std::string uid = sm_get_uid();
	printk("sm>DWM1001-Dev nRF-UID (%s)\n",uid.c_str());

	sm_set_callback_rx_json(rx_topic_json_handler);
	sm_start();

	int loop = 0;
	json j_mesh_state;
	while (1) {
		j_mesh_state["mesh_loop"] = loop;
		mesh_bcast_json(j_mesh_state);
		printf("sm> %s\n",j_mesh_state.dump().c_str());
		k_sleep(K_SECONDS(g_mesh_alive_loop_sec));
		loop++;
	}
}

void uwb_thread(void)
{
	static dwt_config_t config = {5,DWT_PRF_64M,DWT_PLEN_128,DWT_PAC8,9,9,1,DWT_BR_6M8,DWT_PHRMODE_EXT,(129)};

	mp_start(config);
	mp_conf_to_json(config,jconfig);
	mesh_bcast_json(jconfig);
	printf("uwb_config>%s\n",jconfig.dump().c_str());

	int cmd_count = 0;
    while (1) {
		k_sem_take(&sem_uwb_cmd,K_FOREVER);
		if(uwb_cmd.compare("uwb_config") == 0){
			for (auto& [key, value] : j_uwb_cmd.items()) {
				jconfig[key] = value;
			}
			mp_json_to_conf(jconfig,config);
			dwt_configure(&config);
		    printf("new dwt config :\n%s\n",jconfig.dump(2).c_str());
			mesh_bcast_json(jconfig);
		}
		cmd_count++;
    }
}
