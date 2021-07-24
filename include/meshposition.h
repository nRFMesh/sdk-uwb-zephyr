
#include <drivers/dw1000/deca_device_api.h>
#include <json.hpp>
#include <cstdint>

using json = nlohmann::json;

enum struct msg_id_t: uint8_t{
    ping                = 1,    //
    reset               = 2,    //no payload
    alive               = 3,    //uint32_t counter
    address_request     = 4,    //
    address_assign      = 5,    //uint8_t address
    string              = 6,    //std::string
    twr_1_poll          = 7,    // struct
    twr_2_resp          = 8,    // struct
    twr_3_final         = 9,    // struct
    ack                 = 255   //
};

typedef struct
{
    msg_id_t id;
    uint8_t sequence;
    uint8_t source;
    uint8_t dest;
}header_t;

typedef struct
{
    header_t header;
    uint16_t crc;
}msg_header_t;

typedef struct
{
    header_t header;
    uint32_t poll_tx_ts;
    uint32_t resp_rx_ts;
    uint32_t final_tx_ts;
    uint16_t crc;
}msg_twr_final_t;

void mp_start();

void mp_conf_to_json(dwt_config_t &conf,json &jconf);
void mp_json_to_conf(json &jconf,dwt_config_t &conf);

json mp_status_to_json(uint32_t status_reg);
void mp_status_print(uint32_t status_reg);

void mp_start(dwt_config_t &config);
uint32_t mp_get_status();
void mp_rx_now(uint16_t timeout = 0);
void mp_rx_after_tx(uint32_t delay_us,uint16_t timeout);

void mp_request(msg_header_t &header);
void mp_request(uint8_t* data, uint16_t size);

bool mp_receive(msg_id_t id);
bool mp_receive(msg_id_t id,msg_header_t&header);
bool mp_receive(msg_id_t id,msg_twr_final_t& final_msg);
bool mp_receive(uint8_t* data, uint16_t expected_size);

void mp_send(uint8_t* data, uint16_t size);
bool mp_send_at(uint8_t* data, uint16_t size, uint64_t tx_time, uint8_t flag=0);
bool mp_request_at(uint8_t* data, uint16_t size, uint64_t tx_time);

//low level API
uint32_t mp_poll_rx();
uint64_t get_tx_timestamp_u64(void);
uint64_t get_rx_timestamp_u64(void);

//-------------------------------------  twr -------------------------------------
void uwb_ping(uint8_t sequence,uint8_t pinger,uint8_t target);
void uwb_ping_rx(uint8_t sequence,uint8_t pinger,uint8_t target,json &res);
void uwb_cir_acc(uint8_t source);

#ifdef CONFIG_MP_GPIO_DEBUG
    void twr_gpio_init(const struct device *gpio_dev);
#endif
void twr_respond(uint8_t sequence,uint8_t source_initiator,uint8_t dest_responder,json &res);
void twr_intiate(uint8_t sequence,uint8_t source_initiator,uint8_t dest_responder);
