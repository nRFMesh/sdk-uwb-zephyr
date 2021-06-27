/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   Double-sided two-way ranging (DS TWR) responder example code
 *
 *           This is a simple code example which acts as the responder in a 
 *           DS TWR distance measurement exchange. This application waits for 
 *           a "poll" message (recording the RX time-stamp of the poll) 
 *           expected from the "DS TWR initiator" example code (companion to 
 *           this application), and then sends a "response" message recording
 *           its TX time-stamp, after which it waits for a "final" message from
 *           the initiator to complete the exchange. The final message contains
 *           the remote initiator's time-stamps of poll TX, response RX and 
 *           final TX. With this data and the local time-stamps, (of poll RX,
 *           response TX and final RX), this example application works out a 
 *           value for the time-of-flight over-the-air and, thus, the 
 *           estimated distance between the two devices, which it writes 
 *           to console.
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 * Copyright 2019 (c) Frederic Mes, RTLOC.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include <stdio.h>
#include <string.h>

#include <meshposition.h>

#include <drivers/dw1000/deca_device_api.h>
#include <drivers/dw1000/deca_regs.h>
#include <drivers/dw1000/deca_spi.h>
#include <drivers/dw1000/port.h>

// zephyr includes
#include <zephyr.h>
#include <sys/printk.h>

#include <drivers/gpio.h>

#define LOG_LEVEL 3
#include <logging/log.h>
LOG_MODULE_REGISTER(main);

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

/* Example application name and version to display on console. */
#define APP_HEADER "\nDWM1001 & Zephyr\n"
#define APP_NAME "Example 5b - DS TWR RESP\n"
#define APP_VERSION "Version - 1.3\n"
#define APP_LINE "=================\n"

/* Default communication configuration. */
static dwt_config_t config = {5, DWT_PRF_64M, DWT_PLEN_128, DWT_PAC8, 9, 9, 1, DWT_BR_6M8, DWT_PHRMODE_EXT, (129) };

#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8_t rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define ALL_MSG_COMMON_LEN 10

/* Index to access some of the fields in the frames involved 
 * in the process.
 */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4

/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;
static uint8_t frame_seq_nb_rx = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed 
 * to handle.
 */
#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be 
 * examined at a debug breakpoint.
 */
static uint32_t status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion 
 * factor.
 * 1 uus = 512 / 499.2 usec and 1 usec = 499.2 * 128 dtu.
 */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */

/* This is the delay from Frame RX timestamp to TX reply timestamp used for 
 * calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration.
 */
#define POLL_RX_TO_RESP_TX_DLY_UUS 6000

/* This is the delay from the end of the frame transmission to the enable 
 * of the receiver, as programmed for the DW1000's wait for response feature.
 */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500

/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 10000

/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 30

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference so 
 * that it can be examined at a debug breakpoint.
 */
static double tof;
static double distance;

/* String used to display measured distance on console. */
char dist_str[32] = {0};

/* Declaration of static functions. */
static uint64_t get_tx_timestamp_u64(void);
static uint64_t get_rx_timestamp_u64(void);
static void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);


#define STACKSIZE 2048

void responder_thread();
K_THREAD_DEFINE(responder_main, STACKSIZE, responder_thread, NULL, NULL, NULL, 99, 0, 0);

void uwb_start()
{
    openspi();
    port_set_dw1000_slowrate();

    LOG_INF("responder_thread> starting");
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        LOG_ERR("dwt_initialise failed");
        return;
    }
    port_set_dw1000_fastrate();

    dwt_configure(&config);

    /* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set preamble timeout for expected frames. See NOTE 6 below. */
    //NOTE: no use of preamble tmo's yet
    // dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    /* Configure DW1000 LEDs */
    dwt_setleds(1);
}

void responder_thread(void)
{
	gpio_pin_init();
	APP_CLEAR;
	APP_SET;
	APP_CLEAR;
    uwb_start();

    k_yield();

    while (1) {
        //APP_SET_CLEAR 
        // - pulse1: 'pending 1st reception' ; 
        // - pulse2: 'tx resp till rx final'
        // - pulse3: 'computing distance'
        uint32_t reg1 = dwt_read32bitreg(SYS_STATUS_ID);
        dwt_setrxtimeout(0); // 0 : disable timeout
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        APP_SET;
        /* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
        while   ( !(    (status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & 
                        (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)
                    ))
        { };
        APP_CLEAR;
        if (status_reg & SYS_STATUS_RXFCG) {    //Receiver FCS Good
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);//clear flag

            /* A frame has been received, read it into the local buffer. */
            uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= RX_BUFFER_LEN) {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            rx_buffer[ALL_MSG_SN_IDX] = 0;  //ignore sequence number in the next compare
            if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) {//is it poll (step1) msg ?

                /* Retrieve poll reception timestamp. */
                poll_rx_ts = get_rx_timestamp_u64();

                /* Retreive frame sequence number */
                memcpy(&frame_seq_nb_rx, &rx_buffer[2], 1);

                /* Set send time for response. See NOTE 9 below. */
                uint32_t resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(resp_tx_time);

                /* Set expected delay and timeout for final message reception.
                 * See NOTE 4 and 5 below.
                 */
                dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
                dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

                /* Write and send the response message. See NOTE 10 below.*/
                tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

                /* Zero offset in TX buffer. */
                dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); 
                
                /* Zero offset in TX buffer, ranging. */
                dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); 
                APP_SET;
                int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

                /* If dwt_starttx() returns an error, abandon this ranging 
                 * exchange and proceed to the next one. See NOTE 11 below.
                 */
                if (ret == DWT_ERROR) {
                    printk("err - tx_error1\n");
                    APP_CLEAR;
                    continue;
                }

                /* Poll for reception of expected "final" frame or error/timeout. 
                 * See NOTE 8 below.
                 */
                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & 
                    (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
                { };
                APP_CLEAR;
                /* Increment frame sequence number after transmission of the 
                 * response message (modulo 256).
                 */
                frame_seq_nb++;

                if (status_reg & SYS_STATUS_RXFCG) {
                    /* Clear good RX frame event and TX frame sent in the 
                     * DW1000 status register.
                     */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

                    /* A frame has been received, read it into the local buffer. */
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                    if (frame_len <= RX_BUF_LEN) {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }

                    /* Check that the frame is a final message sent by "DS TWR
                     * initiator" example.
                     * As the sequence number field of the frame is not used 
                     * in this example, it can be zeroed to ease the validation
                     * of the frame.
                     */
                    rx_buffer[ALL_MSG_SN_IDX] = 0;
                    if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0) {
                        APP_SET;
                        uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
                        uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                        double Ra, Rb, Da, Db;
                        int64 tof_dtu;

                        /* Retrieve response transmission and final reception 
                         * timestamps.
                         */
                        resp_tx_ts = get_tx_timestamp_u64();
                        final_rx_ts = get_rx_timestamp_u64();

                        /* Get timestamps embedded in the final message. */
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                        /* Compute time of flight. 32-bit subtractions give 
                         * correct answers even if clock has wrapped. 
                         * See NOTE 12 below.
                         */
                        poll_rx_ts_32 = (uint32)poll_rx_ts;
                        resp_tx_ts_32 = (uint32)resp_tx_ts;
                        final_rx_ts_32 = (uint32)final_rx_ts;
                        
                        Ra = (double)(resp_rx_ts - poll_tx_ts);
                        Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                        Da = (double)(final_tx_ts - resp_rx_ts);
                        Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                        tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                        tof = tof_dtu * DWT_TIME_UNITS;
                        distance = tof * SPEED_OF_LIGHT;
                        APP_CLEAR;
                        /* Display computed distance on console. */
                        sprintf(dist_str, "dist (%u): %3.2lf m\n",frame_seq_nb_rx, distance);
                        printk("%s", dist_str);
                    }
                }
                else {
                    printk("error - rx2 failed  %08x\n", status_reg);

                    /* Clear RX error/timeout events in the DW1000 
                     * status register.
                     */
                    dwt_write32bitreg(SYS_STATUS_ID, 
                        SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

                    /* Reset RX to properly reinitialise LDE operation. */
                    dwt_rxreset();
                }
            }
        }
        else {
            dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);//clear errors
            dwt_rxreset();
        }
        uint32_t reg2 = dwt_read32bitreg(SYS_STATUS_ID);
        printk("sequence(%u) over; reg1 = 0x%08x ; reg2 = 0x%08x\n",frame_seq_nb,reg1,reg2);
        mp_status_print(reg2);
        k_sleep(K_MSEC(100));
    }
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
static uint64_t get_tx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;

    dwt_readtxtimestamp(ts_tab);
    for (int i = 4; i >= 0; i--) {
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
 *             for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64_t get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;

    dwt_readrxtimestamp(ts_tab);
    for (int i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! --------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. 
 *        In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts)
{
    *ts = 0;
    for (int i = 0; i < FINAL_MSG_TS_LEN; i++) {
        *ts += ts_field[i] << (i * 8);
    }
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete final frame sent by the responder at the
 *    110k data rate used (around 3.5 ms).
 * 6. The preamble timeout allows the receiver to stop listening in situations where preamble is not starting (which might be because the responder is
 *    out of range or did not receive the message to respond to). This saves the power waste of listening for a message that is not coming. We
 *    recommend a minimum preamble timeout of 5 PACs for short range applications and a larger value (e.g. in the range of 50% to 80% of the preamble
 *    length) for more challenging longer range, NLOS or noisy environments.
 * 7. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 9. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to poll RX
 *    timestamp to get response transmission time. The delayed transmission time resolution is 512 device time units which means that the lower 9 bits
 *    of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower 8 bits.
 * 10. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *     automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *     work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 11. When running this example on the EVB1000 platform with the POLL_RX_TO_RESP_TX_DLY response delay provided, the dwt_starttx() is always
 *     successful. However, in cases where the delay is too short (or something else interrupts the code flow), then the dwt_starttx() might be issued
 *     too late for the configured start time. The code below provides an example of how to handle this condition: In this case it abandons the
 *     ranging exchange and simply goes back to awaiting another poll message. If this error handling code was not here, a late dwt_starttx() would
 *     result in the code flow getting stuck waiting subsequent RX event that will will never come. The companion "initiator" example (ex_05a) should
 *     timeout from awaiting the "response" and proceed to send another poll in due course to initiate another ranging exchange.
 * 12. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *     subtraction.
 * 13. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/
