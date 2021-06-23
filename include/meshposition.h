
#include <drivers/dw1000/deca_device_api.h>
#include <json.hpp>
#include <cstdint>

using json = nlohmann::json;

void mp_start();

void mp_conf_to_json(dwt_config_t &conf,json &jconf);
void mp_json_to_conf(json &jconf,dwt_config_t &conf);

json mp_status_to_json(uint32_t status_reg);
void mp_status_print(uint32_t status_reg);

void mp_start(dwt_config_t &config);


uint64_t get_tx_timestamp_u64(void);
uint64_t get_rx_timestamp_u64(void);
