#include <stdint.h>
#include <string.h>
#include <stdbool.h>
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
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
//#include "ble_bas.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_delay.h"
#include "gps.h"
#include "SEGGER_RTT.h"
#include "ADC.h"
#include "I2C.h"
#define GPS_UART_RX 4
//#if defined (UART_PRESENT)
//#include "nrf_uart.h"
//#endif
//#if defined (UARTE_PRESENT)
//#include "nrf_uarte.h"
//#endif
//
//#include "nrf_log.h"
//#include "nrf_log_ctrl.h"
//#include "nrf_log_default_backends.h"
//#include "nrf_delay.h"

bool is_advertising=false;

#define APP_BLE_CONN_CFG_TAG      1                                           /**< A tag identifying the SoftDevice BLE configuration. */

int adv_interval =100;
uint16_t txpower= 0 ;  //[+4,0,-4,-8,-12,-16,-20,-30,-40];
bool gps_enabled=false;
bool is_connect = false;
bool turn_off_cmd=false;    // It is true when a OFF command send by user service.
bool turn_on_cmd=false;    // It is true when a ON command send by user service.
bool PA_enabled_cmd=false;  // it is True if the APP_PA_PIN is enabled
bool is_Night_Mode_cmd=false;
bool uart_disabled = true;
int VBAT;
bool first_use=true;
uint8_t  battery_level;

//int32_t APP_ADV_INTERVAL= MSEC_TO_UNITS(adv_interval, UNIT_0_625_MS); /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

//int32_t APP_ADV_INTERVAL(adv_interval){
//  return (adv_interval) /(0.625);
//}
#define APP_BEACON_INFO_LENGTH    0x18   /**< Total length of information advertised by the Beacon. *///  24 bytes

#define APP_ADV_DATA_LENGTH       0x1F      /**< Length of manufacturer specific data in the advertisement. */

#define APP_COMPANY_IDENTIFIER    0xFFFF /**< Company identifier as per www.bluetooth.org. */

#define APP_BEACON_UUID           0x59, 0x41, 0x48, 0x48, 0x49  /**< Proprietary UUID for Beacon. */

//#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */



#define APP_ADV_DURATION               0 //BOTE 1 seconds OLD 18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(150, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(20000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

//#define LED1_PIN 11


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
//BLE_BAS_DEF(m_bas);
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

//static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
//{
 //   {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
//};
static char gps_test[128] = "000000.000,6325.2920,N,01026.2479,E,1,08,1.12,110.9";
static char packet[128];


#define AT_CMD_DATA					3

static ble_gap_adv_params_t m_adv_params;                     /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                           /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                            /**< Buffer for storing an encoded advertising set. */
//static uint8_t m_enc_scandata[BLE_GAP_SCAN_BUFFER_EXTENDED_MIN];                            /**< Buffer for storing an encoded advertising set. */
typedef enum
{
    SELECTION_CONNECTABLE = 0, 
    SELECTION_NON_CONNECTABLE
} adv_scan_type_seclection_t;

typedef enum
{
    SELECTION_1M_PHY = 0, 
    SELECTION_CODED_PHY
} adv_scan_phy_seclection_t;

static uint8_t m_adv_scan_type_selected = SELECTION_CONNECTABLE;
static uint8_t m_adv_scan_phy_selected = SELECTION_1M_PHY; 
/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0

    }
};
/**< Heart rate measurement timer. */
APP_TIMER_DEF(timer_id);  
APP_TIMER_DEF(gps_timer_id);  
/**< Heart rate measurement interval (ticks). */
#define TIMER_INTERVAL         APP_TIMER_TICKS(30000)         //500ms  


#define DEVICE_NAME                     "BOTE_V5"//"BlueNor 52832X"                               /**< Name of device. Will be included in the advertising data. */


#define APP_PA_PIN              	17
#define APP_LNA_PIN              	19
//#define APP_PA_PIN              24
//#define APP_LNA_PIN             25
#define APP_CPS_PIN			6
#define APP_AMP_PPI_CH_ID_SET   0
#define APP_AMP_PPI_CH_ID_CLR   1
#define APP_AMP_GPIOTE_CH_ID    0
#define LED 13
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
static void nus_data_handler(ble_nus_evt_t * p_evt);
static void advertising_init(void);
static void advertising_start(void);
static void init_ble_stack_service();

/**@brief Function for updating the Battery Level measurement*/
//static void battery_level_update(void)
//{
//    ret_code_t err_code;
//
//    uint8_t  battery_level;
//    uint16_t vbatt;              // Variable to hold voltage reading
//    //battery_voltage_get(&vbatt); // Get new battery voltage
//    //battery_level = battery_level_in_percent(vbatt);          //Transform the millivolts value into battery level percent.
//    //from Lenar code
//    //VBAT = GetBatteryVoltage1();
//    //battery_level=(VBAT*4583)/100;
//    battery_level=70;
//
////    printf("ADC result in percent: %d\r\n", battery_level);
////
////    err_code = ble_bas_battery_level_update(&m_bas, battery_level, m_conn_handle);
////    if ((err_code != NRF_SUCCESS) &&
////        (err_code != NRF_ERROR_INVALID_STATE) &&
////        (err_code != NRF_ERROR_RESOURCES) &&
////        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
////       )
////    {
////        APP_ERROR_HANDLER(err_code);
////    }
//}


static void pa_lna_setup(void)
{
    uint32_t err_code;
    nrf_gpio_cfg_output(APP_CPS_PIN);
		nrf_gpio_pin_clear(APP_CPS_PIN); //enable
		//nrf_gpio_pin_set(APP_CPS_PIN); //bypass
	  nrf_gpio_cfg_output(APP_PA_PIN);
		nrf_gpio_pin_clear(APP_PA_PIN); //
	  nrf_gpio_cfg_output(APP_LNA_PIN);
		nrf_gpio_pin_clear(APP_LNA_PIN); //
		//nrf_gpio_pin_clear(APP_LNA_PIN);

    static ble_opt_t pa_lna_opts = {
        .common_opt = {
            .pa_lna = {
							
                .pa_cfg = {
                    .enable = 0,
                    .active_high = 1,
                    .gpio_pin = APP_PA_PIN
                },
							
								
                .lna_cfg = {
                    .enable = 0,
                    .active_high = 1,
                    .gpio_pin = APP_LNA_PIN
                },
								
                .ppi_ch_id_set = APP_AMP_PPI_CH_ID_SET,
                .ppi_ch_id_clr = APP_AMP_PPI_CH_ID_CLR,
                .gpiote_ch_id = APP_AMP_GPIOTE_CH_ID
            }
        }
    };
		
    //NRF_GPIO->DIRSET |= (1 << APP_PA_PIN) | (1 << APP_LNA_PIN) ;
    err_code = sd_ble_opt_set(BLE_COMMON_OPT_PA_LNA, &pa_lna_opts);
    APP_ERROR_CHECK(err_code);

}

///////

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

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
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


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
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
/*
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

*/
/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
 /*
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
            //sleep_mode_enter();
						NVIC_SystemReset();	
            break;
        default:
            break;
    }
}
*/




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
        SEGGER_RTT_printf(0,"Data len is set to 0x%X(%d) \n", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    SEGGER_RTT_printf(0,"ATT MTU exchange completed. central 0x%x peripheral 0x%x \n",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}



///**@brief Function for initializing the nrf log module.
// */
//static void log_init(void)
//{
//    ret_code_t err_code = NRF_LOG_INIT(NULL);
//    APP_ERROR_CHECK(err_code);
//
//    NRF_LOG_DEFAULT_BACKENDS_INIT();
//}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
//    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    nrf_pwr_mgmt_run();
    
    //BOTE put system shut down
    //sd_power_system_off();

}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    //ble_bas_init_t     bas_init;
    nrf_ble_qwr_init_t qwr_init = {0};

  if(!turn_off_cmd && !turn_on_cmd && !gps_enabled){
    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
    }
    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

}

static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] = /**< Information advertised by the Beacon. */
    {
        APP_BEACON_UUID
};

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;
    int8_t rssi=0;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            SEGGER_RTT_printf(0,"Connected \n");
            is_connect = true;
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            //sd_ble_gap_rssi_start(m_conn_handle, 0 , 0);
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            if(uart_disabled){
            uart_init(GPS_UART_RX);
            uart_disabled=false;
            }
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            SEGGER_RTT_printf(0,"Disconnected \n");
            is_connect = false;
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            advertising_init();
            advertising_start();
            //raafat
            //NVIC_SystemReset();	
            break;
        /*case BLE_GAP_EVT_ADV_REPORT :
            if(p_ble_evt->evt.gap_evt.params.adv_report.rssi>-21)
            {SEGGER_RTT_printf(0,"Strong signal");}
            else { SEGGER_RTT_printf(0,"Weak signal");}
            SEGGER_RTT_printf(0,"RSSI= %d dBm \n", p_ble_evt->evt.gap_evt.params.adv_report.rssi);
            SEGGER_RTT_printf(0,"Power= %d  \n", p_ble_evt->evt.gap_evt.params.adv_report.tx_power);
            break;*/

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            SEGGER_RTT_printf(0,"PHY update request. \n");
				
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
				
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

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

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for setting up advertising data. */
static void advertising_init(void)
{

    ret_code_t ret;
    ble_advdata_t advdata;

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    ble_gap_adv_params_t m_adv_params =
    {
        .properties    =
        {
          .type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED,
        },
        .p_peer_addr   = NULL,
        .filter_policy = BLE_GAP_ADV_FP_ANY,
        .interval      = adv_interval /(0.625),
        .duration      = 0,

        .primary_phy   = BLE_GAP_PHY_1MBPS, // Must be changed to connect in long range. (BLE_GAP_PHY_CODED)
        .secondary_phy = BLE_GAP_PHY_1MBPS,
        .scan_req_notification = 1,
    };

    if(m_adv_scan_phy_selected == SELECTION_1M_PHY)
    {
        // 1M coded for adv
        SEGGER_RTT_printf(0,"Setting adv params PHY to 1M .. \n");
        m_adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
        m_adv_params.secondary_phy   = BLE_GAP_PHY_1MBPS;
        
        if(m_adv_scan_type_selected == SELECTION_CONNECTABLE)
        {
            SEGGER_RTT_printf(0,"Advertising type set to CONNECTABLE_SCANNABLE_UNDIRECTED \n");
            m_adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
        }
        else if(m_adv_scan_type_selected == SELECTION_NON_CONNECTABLE)
        {
            SEGGER_RTT_printf(0,"Advertising type set to NONCONNECTABLE_SCANNABLE_UNDIRECTED \n");
            m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;
        }
      }
      /////// BLE advertising data
      ble_advdata_manuf_data_t manuf_specific_data;

      manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
      get_gps_local(&m_beacon_info[0]);
      manuf_specific_data.data.p_data = (uint8_t *)m_beacon_info;
      printf((uint8_t *)m_beacon_info);
      manuf_specific_data.data.size = APP_BEACON_INFO_LENGTH;

      // Build and set advertising data.
      memset(&advdata, 0, sizeof(advdata));

      advdata.name_type          = BLE_ADVDATA_NO_NAME,
      advdata.p_manuf_specific_data = &manuf_specific_data;
      advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
      advdata.include_appearance = false;
        
        ret = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
        APP_ERROR_CHECK(ret);

        ret = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
        APP_ERROR_CHECK(ret);
    

}


/**@brief Function for starting advertising. */
static void advertising_start(void)
{
    SEGGER_RTT_printf(0,"Starting advertising. \n");
    ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV,m_advertising.adv_handle,txpower);
    APP_ERROR_CHECK(err_code);
    is_advertising=true;
    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
}

static void advertising_stop(void)
{
    ret_code_t err_code;
    if (is_advertising){
      err_code = sd_ble_gap_adv_stop(m_adv_handle);
      APP_ERROR_CHECK(err_code);
      is_advertising=false;
      SEGGER_RTT_printf(0,"advertising stopped\n");
    }
}


static uint8_t cnt =0x30;

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        //SEGGER_RTT_printf(0,"Received data from BLE NUS. \n");
        //SEGGER_RTT_printf(0,p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    SEGGER_RTT_printf(0,"Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }

        SEGGER_RTT_printf(0,"Received data from BLE NUS "+p_evt->params.rx_data.length);//params.rx_data.p_data[0]
//        if(p_evt->params.rx_data.length == 2){
//            SEGGER_RTT_printf(0,"Here i am nus \n");
//            if(p_evt->params.rx_data.p_data[0] == 'g' && p_evt->params.rx_data.p_data[1] == '0'){
//		SEGGER_RTT_printf(0,"TURN OFF GPS. \n");
//                turn_Off_GPS();
//                adv_interval=3000;
//                        
//            }else if(p_evt->params.rx_data.p_data[1] == '1'){
//                SEGGER_RTT_printf(0,"TURN ON GPS. \n");
//                turn_ON_GPS();
//                }
//	}	

          if(p_evt->params.rx_data.length == 1){
           if(p_evt->params.rx_data.p_data[0] == 0x0){
                SEGGER_RTT_printf(0,"TURN OFF GPS. \n");
                gps_enabled=false;
                turn_off_cmd=true;
                // it happens user clicks button on and of several times.\
                 //this  to make sure to execute ONLY the last status
                turn_on_cmd=false; 

                //turn_Off_GPS();
                //advertising_stop();
                //advertising_init();
                //advertising_start();
                
            }
            if(p_evt->params.rx_data.p_data[0] == 0x1){
                SEGGER_RTT_printf(0,"TURN ON GPS. \n");
                gps_enabled=true;
                turn_on_cmd=true;
                // it happens user clicks button on and of several times.\
                 //this  to make sure to execute ONLY the last status
                turn_off_cmd=false; 
            
            }

            // System Reset 
            if(p_evt->params.rx_data.p_data[0] == 0x2){
            SEGGER_RTT_printf(0,"System RESET. \n");
              NVIC_SystemReset();
            }
          

            // NIGHT MODE 
            if(p_evt->params.rx_data.p_data[0] == 0x3){
              SEGGER_RTT_printf(0,"NIGHT MODE. \n");
              is_Night_Mode_cmd=true;
            }

            // Toggle PA_LNA
            if(p_evt->params.rx_data.p_data[0] == 0x4){
              SEGGER_RTT_printf(0,"PA Toggling. \n");
              PA_enabled_cmd=true;
              nrf_gpio_pin_toggle(APP_PA_PIN);  
            }
          
      }
      }
}

/**@snippet [Handling the data received over BLE] */

/*static void gps_timer_handler( void * p_context)
{
    if(turn_on_cmd)
    {
    SEGGER_RTT_printf(0,"gps timer expired \n");
    advertising_stop();
    SEGGER_RTT_printf(0,"advertisement stopped \n");
    turn_ON_GPS();
    SEGGER_RTT_printf(0,"GPS turned ON \n");
    app_timer_stop(gps_timer_id);
    nrf_delay_ms(145000);
    app_timer_start(timer_id, TIMER_INTERVAL, NULL);
    }
    if(turn_off_cmd)
    {
    turn_Off_GPS();
    advertising_stop();
    adv_interval=2000;
    txpower=-8;
    advertising_init();
    advertising_start();
    }*/
    /*if(first_use) 
    {
       nrf_gpio_pin_set(LED);
       nrf_delay_ms(40000);
       nrf_gpio_pin_clear(LED);
       first_use=false;
    }
    nrf_delay_ms(10000);
    nrf_gpio_pin_set(LED);
    nrf_delay_ms(10000);
    nrf_gpio_pin_clear(LED);*/

//}
    
static void timer_handler(void * p_context)
{ 
    if(turn_on_cmd)
    {
      if (first_use)
      {
        SEGGER_RTT_printf(0,"gps signal acquisition \n");
        advertising_stop();
        turn_ON_GPS();
        SEGGER_RTT_printf(0,"gps activate \n");
        app_timer_stop(timer_id);
        nrf_delay_ms(150000);
        first_use=false;
        app_timer_start(timer_id,TIMER_INTERVAL,NULL);
      }
      SEGGER_RTT_printf(0,"After acquisition \n");
      turn_Off_GPS();
      nrf_gpio_pin_set(APP_PA_PIN);
      nrf_gpio_pin_set(APP_LNA_PIN);
      advertising_init();
      advertising_start();
      nrf_delay_ms(20000);
      SEGGER_RTT_printf(0,"Turn on again \n");
      advertising_stop();
      SEGGER_RTT_printf(0,"Here \n");
      turn_ON_GPS();
    }
    //nrf_gpio_pin_set(APP_PA_PIN);
    //nrf_gpio_pin_set(APP_LNA_PIN);
    /*SEGGER_RTT_printf(0,"start advertising \n");
    adv_interval=100;*/
    //advertising_init();
    //advertising_start();
    /*advertising_stop();
    nrf_delay_ms(15000);*/ 
    // Turning off the PA\LNA module, stopping advertisement and GPS tracking
    /*SEGGER_RTT_printf(0,"gps signal tracking \n");
    turn_Off_GPS();
    nrf_gpio_pin_set(LED);
    //advertising_stop(); 
    advertising_init();
    advertising_start();
    SEGGER_RTT_printf(0,"Stop advertising \n");
    //turn_Off_GPS();
    SEGGER_RTT_printf(0,"Turn ON the GPS \n");*/
    //}
    if(turn_off_cmd)
    {
      turn_Off_GPS();
      advertising_stop();
      nrf_gpio_pin_clear(LED);
      //nrf_gpio_pin_clear(APP_PA_PIN);
      //nrf_gpio_pin_clear(APP_LNA_PIN);
      adv_interval=2000;
      txpower=0;
      advertising_init();
      advertising_start();
      first_use=true;
    }
    /*
    //Turn off the GPS and start advertising
    turn_Off_GPS();
    advertising_init();
    advertising_start();
    //nrf_gpio_pin_set(LED);
    nrf_delay_ms(20000);
    }*/
    /*
    //char nus_message[35]="" ;
  char* gps_status;  //,adv_intval="adv_intval_";

  //SEGGER_RTT_printf(0,"\r\nTimer handler\r\n");

  SEGGER_RTT_printf(0,"Wakeup pin status %d\n",nrf_gpio_pin_read(7));
  nrf_gpio_cfg_input(23,NRF_GPIO_PIN_NOPULL);
  SEGGER_RTT_printf(0,"Charging status %d\n",!nrf_gpio_pin_read(23));
  SEGGER_RTT_printf(0,"TX POWER  %d\n",txpower);
  if(gps_enabled) {SEGGER_RTT_printf(0,"gps_enabled is TRUE\n"); }else{SEGGER_RTT_printf(0,"gps_enabled is FALSE\n"); }
  if(is_gps_fixed()) {SEGGER_RTT_printf(0,"is_gps_fixed is TRUE\n"); }else{SEGGER_RTT_printf(0,"is_gps_fixed is FALSE\n"); }
  // in the connected mode
  if(is_connect)
  {
    SEGGER_RTT_printf(0,"Connected Timer handler\n");
    VBAT = GetBatteryVoltage1();
    int VBAT1=VBAT;
    SEGGER_RTT_printf(0,"Battery level as integer %d\n",VBAT1);
    int i=4;
    uint8_t nus_message[10]="";
    // splitting the VBAT into digits each
    while (i>=0)
    {
      //SEGGER_RTT_printf(0,"%d\n", VBAT1 % 10);
      nus_message[i] = VBAT1 % 10;
      //SEGGER_RTT_printf(0,"%d\n", nus_message[i]);
      VBAT1 /= 10;
      i--;
    }
  
    SEGGER_RTT_printf(0,"battery level message %d%d%d%d%d\n", nus_message[0], nus_message[1],nus_message[2],
    nus_message[3],nus_message[4]);

    // PA PIN STATUS
    if (nrf_gpio_pin_read(APP_PA_PIN)==1) nus_message[6]=0x01 ;else nus_message[6]=0x00;

    //GPS status
    if (nrf_gpio_pin_read(7)==1) nus_message[7]=0x01 ;else nus_message[7]=0x00;

    // YES CHARGING status equal 0 when it is charging 
    if (nrf_gpio_pin_read(23)==0) nus_message[8]=0x01 ;else nus_message[8]=0x00;

      nus_message[9]=cnt;   //  adding counter to the last byte  of the message 
      cnt++;
      if(cnt > 0x39)  cnt = 0x30;
      //gps_test[63] = cnt;
      //packet[1] = VBAT;
      //packet[4] = cnt;
      uint16_t length = 10;
      //ble_nus_data_send(&m_nus, packet, &length,m_conn_handle);
      //nrf_gpio_cfg_input(7,NRF_GPIO_PIN_PULLUP);
      //if (nrf_gpio_pin_read(7)==1) nus_message[2]="0x1" ;else nus_message[2]="0x0" ;
      //nus_message[5] = VBAT*5;
      //strcat(adv_intval, adv_interval);
      //sprintf(nus_message, "%s",gps_status);//,adv_interval);

      ble_nus_data_send(&m_nus, nus_message, &length,m_conn_handle);
 } */
 }
   // }// end of if connected

   /* else
    { //not conencted 

      //  after disconnect and with turn off mode , the system disables the PA and reinit advertising
      if(turn_off_cmd){      
        SEGGER_RTT_printf(0,"Off command sent by the user \n");
        //advertising_stop();
        //disable PA
        nrf_gpio_pin_clear(APP_PA_PIN);
        nrf_gpio_pin_clear(APP_LNA_PIN);
        advertising_stop();
        //uint32_t err_code = sd_ble_opt_set(BLE_COMMON_OPT_PA_LNA, NULL);
        ret_code_t err_code = nrf_sdh_disable_request();
        APP_ERROR_CHECK(err_code);
        init_ble_stack_service();;   // reinit ble stack and its services.
        adv_interval=4000;
        txpower= -20;
        advertising_init();
        advertising_start();
        turn_off_cmd=false;
      }*/

       //  after disconnect and with turn ON mode, the system will reinit advertising.
     // if(turn_on_cmd)
      //{      
        /*SEGGER_RTT_printf(0,"ON command sent by the user \n");
      
        advertising_stop();
        
        turn_ON_GPS();  
        //ret_code_t err_code = nrf_sdh_disable_request();
        //APP_ERROR_CHECK(err_code);
        //nrf_delay_ms(10);
        //init_ble_stack_service();  // reinit ble stack and its services.
        turn_on_cmd=false;
        //pa_lna_setup();             //Enable the PA 
        adv_interval=2000;
        txpower= 4;
        advertising_init();
        advertising_start();  */
   /* if(turn_on_cmd)
    {
    //start advertising and turning off the GPS after two minutes from activating it
    SEGGER_RTT_printf(0,"turning off GPS \n");
    turn_Off_GPS();
    SEGGER_RTT_printf(0,"start advertising \n");
    pa_lna_setup();
    SEGGER_RTT_printf(0,"Activate the long range module \n");
    advertising_init();
    advertising_start();
    nrf_gpio_pin_set(LED);
    nrf_delay_ms(20000); //30000

    // Turning off the PA\LNA module, stopping advertisement and turning On the GPS
    advertising_stop(); 
    SEGGER_RTT_printf(0,"Stop advertising \n");
    turn_ON_GPS();
    SEGGER_RTT_printf(0,"Turn ON the GPS \n");
    nrf_gpio_pin_clear(LED);
    nrf_delay_ms(10000); //5000
      
    } */
      //If PA_toggleing command is sent 
     /* if(PA_enabled_cmd){
         SEGGER_RTT_printf(0,"APP_PA_PIN pin status %d\n",nrf_gpio_pin_read(APP_PA_PIN));
         PA_enabled_cmd=false;
         advertising_stop();
         advertising_init();
         advertising_start();
      }

      //If is_Night_mode command is sent 
      if(is_Night_Mode_cmd){
         SEGGER_RTT_printf(0,"Is_NIGHT_MODE Enabled");
         is_Night_Mode_cmd=false;
         turn_Off_GPS();
         adv_interval=7000;
         txpower= -20;
         advertising_stop();
         advertising_init();
         advertising_start();
         
      }

      if(!is_gps_fixed() && gps_enabled){
        SEGGER_RTT_printf(0,"Waiting GPS for initial fix.\n");
        //turn_ON_GPS();

        SEGGER_RTT_printf(0,"DisConnected Timer handler\n");
        //txpower=0;  //  was set here because before was not able to communicate after connection closed.
        if(is_advertising)  advertising_stop();
        txpower=0;
        adv_interval=2000;
        //disable PA
        //nrf_gpio_pin_clear(APP_PA_PIN);
        //ret_code_t err_code = nrf_sdh_disable_request();
        //APP_ERROR_CHECK(err_code);
        //init_ble_stack_service();;   // reinit ble stack and its services.
        advertising_init();
        advertising_start();
      }*/


     /* if(is_gps_fixed() &&  gps_enabled)
      {
        //toggle_GPS();
        //SEGGER_RTT_printf(0,"GPS toggeling.\n");
        //nrf_delay_ms(10000);
        //turn_Off_GPS();
        //gps_enabled=false;
        //SEGGER_RTT_printf(0,"GPS is %d",gps_enabled);
        /// Enableing PA with Tx power +4 when 
        //nrf_gpio_cfg_input(7,NRF_GPIO_PIN_PULLUP);	
        //if (nrf_gpio_pin_read(7) == 0){  // when GPS is OFF enable PA
        //enable PA
        if(is_advertising) advertising_stop();
        //disable Soft device to reinit the stack
        //ret_code_t err_code = nrf_sdh_disable_request();
        //APP_ERROR_CHECK(err_code);
        //nrf_delay_ms(10);
        //init_ble_stack_service();  // reinit ble stack and its services.
        //pa_lna_setup();             //Enable the PA 
        txpower= 4;
        adv_interval=1000;
        //APP_ERROR_CHECK(err_code);
        //if(is_advertising)  advertising_stop();
        advertising_init();
        advertising_start();
        }*/
      //}
    

      //nrf_gpio_pin_toggle(APP_PA_PIN);
      //advertising_stop();
      //advertising_init();
      //advertising_start();



// initialise BLE services
static void init_ble_stack_service(){
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    m_adv_scan_phy_selected = SELECTION_1M_PHY;
    conn_params_init();
    }

/**@brief Application main function.
 */
int main(void)
  {
    bool erase_bonds;
    
    //Initialize the LED to be off
    nrf_gpio_cfg_output(LED);
    nrf_gpio_pin_clear(LED);
    //gps_on_pin
    nrf_gpio_cfg_output(11);
    nrf_gpio_pin_clear(11);
       
    //timer initialization
    timers_init();

    //init ble stack and services
    init_ble_stack_service();
   
     turn_Off_GPS();
     pa_lna_setup();
     
    advertising_init();
    advertising_start();
    uint32_t err_code = app_timer_create(&timer_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_handler);
    err_code = app_timer_start(timer_id, TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);	
    // Start execution.
    SEGGER_RTT_printf(0,"APP started.\r\n");  
    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}

/**
 * @}
 */

