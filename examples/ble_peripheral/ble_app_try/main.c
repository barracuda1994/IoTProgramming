/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "bmi160.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"
#include "arm_math.h"
#include "fdacoefs.h"



#define SPI_INSTANCE 0 // SPI instance index. We use SPI master 0

#define SPI_SS_PIN 26
#define SPI_MISO_PIN 23 //5 //SA0
#define SPI_MOSI_PIN 24 //7 //SDA
#define SPI_SCK_PIN 22 //8 //SCL

#define NUM_TAPS 58
#define BLOCK_SIZE 28
// Declare a state array
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
// Declare an instance for the low-pass FIR filter
arm_fir_instance_f32 fir_lpf;
#define SIZE_BUFFER 140

int SENSIBILITY=1250;
int prev_sens= 0;

int steps = 0;

float32_t previous_step;

//SPI instance
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
//Flag used to indicate that SPI instance completed the transfer
static volatile bool spi_xfer_done;
//Sensor variable
struct bmi160_dev sensor;
// Allocate a buffer for SPI reads
static uint8_t SPI_RX_Buffer[140];


  float32_t gyro_x_in_buf[SIZE_BUFFER];
  float32_t gyro_y_in_buf[SIZE_BUFFER];
  float32_t gyro_z_in_buf[SIZE_BUFFER];
	float32_t gyro_x_out_buf[SIZE_BUFFER];
 float32_t gyro_y_out_buf[SIZE_BUFFER];
 float32_t gyro_z_out_buf[SIZE_BUFFER];

 float32_t gyro_tot[SIZE_BUFFER];
 float32_t gyro_tot_out[SIZE_BUFFER];
 uint8_t block_cnt;

// Declare memory to store the raw FIFO buffer information
uint8_t fifo_buff[200];
// Modify the FIFO buffer instance and link to the device instance
struct bmi160_fifo_frame fifo_frame;

#define BUTTON_PIN 11


// 200 bytes -> ~7bytes per frame -> ~28 data frames
struct bmi160_sensor_data gyro_data[28];


#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define DEVICE_NAME                     "Step_counter"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           1                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


BLE_NUS_DEF(m_nus);                                                                 /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */



static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;
			char sensible[5];

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
			
        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
							
							sensible[i]=p_evt->params.rx_data.p_data[i];
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
						
        }
				
				
				if (sensible[0]=='0'){
					steps=0;
				}else{
				SENSIBILITY = atoi(sensible);
				}
				
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length-1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

#if defined(S132)
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
#endif

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

         case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
        {
            ble_gap_data_length_params_t dl_params;

            // Clearing the struct will effectivly set members to @ref BLE_GAP_DATA_LENGTH_AUTO
            memset(&dl_params, 0, sizeof(ble_gap_data_length_params_t));
            err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dl_params, NULL);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, 64);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
             break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
            {
                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                do
                {
                    uint16_t length = (uint16_t)index;
                    err_code = ble_nus_string_send(&m_nus, data_array, &length);
                    if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) )
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                } while (err_code == NRF_ERROR_BUSY);

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}



/**
* Function for writing to the BMI160 via SPI.
*/
int8_t bmi160_spi_bus_write(uint8_t hw_addr, uint8_t reg_addr, uint8_t *reg_data,
uint16_t cnt)
{
 spi_xfer_done = false; // set the flag down during transfer

 int32_t error = 0;
 // Allocate array, which lenght is address + number of data bytes to be sent
 uint8_t tx_buff[cnt+1];

 uint16_t stringpos;
 // AND address with 0111 1111; set msb to '0' (write operation)
 tx_buff[0] = reg_addr & 0x7F;

 for (stringpos = 0; stringpos < cnt; stringpos++) {
 tx_buff[stringpos+1] = *(reg_data + stringpos);
 }
 // Do the actual SPI transfer
 nrf_drv_spi_transfer(&spi, tx_buff, cnt+1, NULL, 0);

 while (!spi_xfer_done) {}; // Loop until the transfer is complete

 return (int8_t)error;
}




/**
* Function for reading from the BMI160 via SPI.
*/
int8_t bmi160_spi_bus_read(uint8_t hw_addr, uint8_t reg_addr, uint8_t *reg_data,
uint16_t len)
{
 spi_xfer_done = false; // set the flag down during transfer
 int32_t error = 0;
 uint8_t tx_buff = reg_addr | 0x80; // OR address with 1000 0000; Read -> set msb to '1';
 uint8_t * rx_buff_pointer;
 uint16_t stringpos;
 rx_buff_pointer = (uint8_t *) (SPI_RX_Buffer);

 // Do the actual SPI transfer
 nrf_drv_spi_transfer(&spi, &tx_buff, 1, rx_buff_pointer, len+1);

 while (!spi_xfer_done) {} // Loop until the transfer is complete
 // Copy received bytes to reg_data
 for (stringpos = 0; stringpos < len; stringpos++)
 *(reg_data + stringpos) = SPI_RX_Buffer[stringpos + 1];

 return (int8_t)error;
}



/**
* Function for computing the filter response
*/
void compute_fir(){

 // Each axis is processed on its own in 5 blocks of data

 // x-axis
 for (uint8_t i = 0; i < 5; i++) {
 arm_fir_f32(&fir_lpf, &gyro_x_in_buf[0] + i * BLOCK_SIZE, &gyro_x_out_buf[0] + i * BLOCK_SIZE, BLOCK_SIZE);
 }
 // y-axis
 for (uint8_t i = 0; i < 5; i++) {
 arm_fir_f32(&fir_lpf, &gyro_y_in_buf[0] + i * BLOCK_SIZE, &gyro_y_out_buf[0] + i * BLOCK_SIZE, BLOCK_SIZE);
 }
 // z-axis
 for (uint8_t i = 0; i < 5; i++) {
 arm_fir_f32(&fir_lpf, &gyro_z_in_buf[0] + i * BLOCK_SIZE, &gyro_z_out_buf[0] + i * BLOCK_SIZE, BLOCK_SIZE);
 }


	//loop for calculating the magnitude of the accelleration
	//for(int i=0; i<140; i++){
	//	gyro_tot[i]= sqrt((gyro_x_in_buf[i]*gyro_x_in_buf[i])+(gyro_y_in_buf[i]*gyro_y_in_buf[i])+(gyro_z_in_buf[i]*gyro_z_in_buf[i]));
	//}
	
	//loop for calculating the magnitude of the accelleration
	for(int i=0; i<140; i++){
		gyro_tot_out[i]= sqrt((gyro_x_out_buf[i]*gyro_x_out_buf[i])+(gyro_y_out_buf[i]*gyro_y_out_buf[i])+(gyro_z_out_buf[i]*gyro_z_out_buf[i]));
	}
	


	float64_t tot_mean = 0;
	//calculate the mean value
	for(int i=28; i<140; i++){
		tot_mean = tot_mean + gyro_tot_out[i];
	}

	float32_t to_remove= tot_mean / 112;

	for(int i=28; i<140; i++){
		gyro_tot_out[i]=gyro_tot_out[i]-to_remove;
	}


	previous_step=0;
	int direction=0;

	//function for counting the peaks
	for(int i=28; i<140; i++){
		//function increasing
		if (direction==0) {
			if (gyro_tot_out[i]<previous_step){
				if (previous_step>SENSIBILITY){
					steps++;
				}
				direction=1;
				previous_step=gyro_tot_out[i];
			} else {
				previous_step=gyro_tot_out[i];
			}
		} else {
			if (gyro_tot_out[i]>previous_step){
			direction=0;
			previous_step=gyro_tot_out[i];
				} else {
					previous_step=gyro_tot_out[i];
		}
	}
	}


 block_cnt = 0; // Reset block counting
}



/**
* Function for reading FIFO data
*/
int8_t get_bmi160_fifo_data()
{
 int8_t rslt = BMI160_OK;
 uint8_t gyro_frames_req = 28;
 // Read the fifo buffer using SPI
 rslt = bmi160_get_fifo_data(&sensor);

 // Parse the data and extract 28 gyro frames
 rslt = bmi160_extract_gyro(gyro_data, &gyro_frames_req, &sensor);

// Copy the contents of each axis to a FIR input buffer
for (uint8_t i = 0; i < gyro_frames_req; i++) {
 gyro_x_in_buf[gyro_frames_req*block_cnt + i] = gyro_data[i].x;
 gyro_y_in_buf[gyro_frames_req*block_cnt + i] = gyro_data[i].y;
 gyro_z_in_buf[gyro_frames_req*block_cnt + i] = gyro_data[i].z;
}

// Increase block count after each sensor read
block_cnt++;

// After 5 reads the buffer is almost full and the data is ready to be processed
if (block_cnt == 5) {
 compute_fir();
}


 return rslt;
}



/**
* SPI user event handler.
*/
void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
 spi_xfer_done = true; // Set a flag when transfer is done
	//call the function for reading the FIFO data.

}



/**
* Function for setting up the SPI communication.
*/
uint32_t spi_config()
{
 uint32_t err_code;

 // Use nRF's default configurations
 nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
 // Define each GPIO pin
 spi_config.ss_pin = SPI_SS_PIN;
 spi_config.miso_pin = SPI_MISO_PIN;
 spi_config.mosi_pin = SPI_MOSI_PIN;
 spi_config.sck_pin = SPI_SCK_PIN;

 // Initialize the SPI peripheral and give it a function pointer to
 // it? event handler
 err_code = nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL);

 return err_code;
}





/**
* Function for configuring the sensor
*/
int8_t sensor_config()
{



	int8_t rslt = BMI160_OK;
 sensor.id = 0; // We use SPI so id == 0
 sensor.interface = BMI160_SPI_INTF;
 // Give the driver the correct interfacing functions
 sensor.read = bmi160_spi_bus_read;
 sensor.write = bmi160_spi_bus_write;
 sensor.delay_ms = nrf_delay_ms;
 // Initialize the sensor and check if everything went ok
 rslt = bmi160_init(&sensor);



	// Configure the gyroscope's sampling freq, range and modes
sensor.gyro_cfg.odr = BMI160_GYRO_ODR_25HZ;
sensor.gyro_cfg.range = BMI160_GYRO_RANGE_1000_DPS;
sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;
// Set the configurations
 rslt = bmi160_set_sens_conf(&sensor);

// Some fifo settings
fifo_frame.data = fifo_buff;
fifo_frame.length = 200;
sensor.fifo = &fifo_frame;

// Configure the sensor's FIFO settings
rslt = bmi160_set_fifo_config(BMI160_FIFO_GYRO, BMI160_ENABLE, &sensor);
// Create an instance for interrupt settings
struct bmi160_int_settg int_config;

// Interrupt channel/pin 1
int_config.int_channel = BMI160_INT_CHANNEL_1;
// Choosing fifo watermark interrupt
int_config.int_type = BMI160_ACC_GYRO_FIFO_WATERMARK_INT;

// Set fifo watermark level to 180
rslt = bmi160_set_fifo_wm((uint8_t) 180, &sensor);

// Enabling interrupt pins to act as output pin
int_config.int_pin_settg.output_en = BMI160_ENABLE;
// Choosing push-pull mode for interrupt pin
int_config.int_pin_settg.output_mode = BMI160_DISABLE;
// Choosing active high output
int_config.int_pin_settg.output_type = BMI160_ENABLE;
// Choosing edge triggered output
int_config.int_pin_settg.edge_ctrl = BMI160_ENABLE;
// Disabling interrupt pin to act as input
int_config.int_pin_settg.input_en = BMI160_DISABLE;
// Non-latched output
int_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_NONE;
// Enabling FIFO watermark interrupt
int_config.fifo_WTM_int_en = BMI160_ENABLE;

// Set interrupt configurations
rslt = bmi160_set_int_config(&int_config, &sensor);



 return rslt;
}


const nrf_drv_timer_t TIMER_TEMP = NRF_DRV_TIMER_INSTANCE(0);

/**
* Handler for GPIO events.
*/
void button_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
 get_bmi160_fifo_data();
}


/**
* Function for configuring General Purpose I/O.
*/
uint32_t config_gpio()
{
 uint32_t err_code = NRF_SUCCESS;
 if(!nrf_drv_gpiote_is_init())
 {
 err_code = nrf_drv_gpiote_init();
 }
 // Set which clock edge triggers the interrupt
 nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
 // Configure the internal pull up resistor
 config.pull = NRF_GPIO_PIN_NOPULL;

 // Configure the pin as input
 err_code = nrf_drv_gpiote_in_init(BUTTON_PIN, &config, button_handler);
 if (err_code != NRF_SUCCESS)
 {
 // handle error condition
 }
 // Enable events
 nrf_drv_gpiote_in_event_enable(BUTTON_PIN, true);
 return err_code;
}

/**
* Function for initializing the FIR filter instance
*/
void dsp_config() {

 // Note that the init function requires the address of the coefficient table B as an input
 arm_fir_init_f32(&fir_lpf, NUM_TAPS, (float32_t *)&B[0], &firStateF32[0],
BLOCK_SIZE);

}

/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    bool     erase_bonds;

    // Initialize.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    uart_init();
    log_init();

    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    printf("\r\nUART Star");
    NRF_LOG_INFO("UART Start");
    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
		
    APP_ERROR_CHECK(err_code);
		
		config_gpio();
	spi_config();
	sensor_config();
	dsp_config();
		
uint16_t length = 0;
char PrintBuffer[128] = {0}; 
    // Enter main loop.

    for (;;)
    {
			power_manage(); 
        memset(PrintBuffer,0x00,sizeof(PrintBuffer));			//clear PrintBuffer buffer
			
			sprintf(PrintBuffer, "The current steps are now:%d\r\n the sensibility is: %d", steps, SENSIBILITY);         //prepare data 
        length = strlen(PrintBuffer);                                   //get data length
	err_code = ble_nus_string_send(&m_nus, PrintBuffer, &length);    //output data via ble
//SENSIBILITY = atoi(p_evt->params.rx_data.p_data);
			if(prev_sens!=SENSIBILITY){
				printf("\n The new sensibility is: %d", SENSIBILITY);
				prev_sens=SENSIBILITY;
			}
        
    }
}


/**
 * @}
 */
