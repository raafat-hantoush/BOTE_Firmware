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
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_delay.h"
#include "gps.h"
#include "SEGGER_RTT.h"
#include "ADC.h"
#include "I2C.h"

#define GPS_UART_RX               4

int txpower = +4;

#define DEVICE_NAME                     "BOTE_V5"
#define APP_ADV_INTERVAL                1120  //BOTE 700 mili seconds  old 64                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION               100 //BOTE 1 seconds OLD 18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(2000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_BLE_CONN_CFG_TAG      1 /**< A tag identifying the SoftDevice BLE configuration. */

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

//#define NON_CONNECTABLE_ADV_INTERVAL MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH    0x1B   /**< Total length of information advertised by the Beacon. */

#define APP_ADV_DATA_LENGTH       0x1B      /**< Length of manufacturer specific data in the advertisement. */

#define APP_COMPANY_IDENTIFIER    0xFFFF /**< Company identifier as per www.bluetooth.org. */

#define APP_BEACON_UUID           0x59, 0x41, 0x48, 0x48, 0x49  /**< Proprietary UUID for Beacon. */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static char gps_test[128] = "000000.000,6325.2920,N,01026.2479,E,1,08,1.12,110.9";
static char packet[128];

static bool is_connect = false;

#define AT_CMD_DATA							3

static ble_gap_adv_params_t m_adv_params;                     /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */

typedef enum
{
    SELECTION_CONNECTABLE = 0, 
    SELECTION_NON_CONNECTABLE
} adv_scan_type_seclection_t;

static uint8_t m_adv_scan_type_selected = SELECTION_CONNECTABLE;

////////////////////  Power Amplifier PA LNA BT832X
#define APP_PA_PIN 17
#define APP_LNA_PIN 19

#define APP_CPS_PIN 6

#define APP_AMP_PPI_CH_ID_SET 0
#define APP_AMP_PPI_CH_ID_CLR 1
#define APP_AMP_GPIOTE_CH_ID 0

static void pa_lna_setup(void) {
  uint32_t err_code;
  nrf_gpio_cfg_output(APP_CPS_PIN);
  nrf_gpio_pin_clear(APP_CPS_PIN); //enable
  nrf_gpio_cfg_output(APP_PA_PIN);
  nrf_gpio_pin_clear(APP_PA_PIN); //
  nrf_gpio_cfg_output(APP_LNA_PIN);
  nrf_gpio_pin_clear(APP_LNA_PIN); //

  static ble_opt_t pa_lna_opts = {
      .common_opt = {
          .pa_lna = {

              .pa_cfg = {
                  .enable = 1,
                  .active_high = 1,
                  .gpio_pin = APP_PA_PIN},

              .lna_cfg = {.enable = 1, .active_high = 1, .gpio_pin = APP_LNA_PIN},

              .ppi_ch_id_set = APP_AMP_PPI_CH_ID_SET,
              .ppi_ch_id_clr = APP_AMP_PPI_CH_ID_CLR,
              .gpiote_ch_id = APP_AMP_GPIOTE_CH_ID}}};

  //NRF_GPIO->DIRSET |= (1 << APP_PA_PIN) | (1 << APP_LNA_PIN) ;
  nrf_gpio_pin_set(APP_PA_PIN);
  err_code = sd_ble_opt_set(BLE_COMMON_OPT_PA_LNA, &pa_lna_opts);
  APP_ERROR_CHECK(err_code);
}

// Battery level
#define ADC12_COUNTS_PER_VOLT 4551
uint8_t bat_volt1;
uint8_t bat_volt2;

/**
 * @brief Function for 14-bit adc init in polled mode
 */
void Adc12bitPolledInitialise(void)
{
//    uint32_t timeout = 10;
    nrf_saadc_channel_config_t myConfig =
    {
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .gain       = NRF_SAADC_GAIN1_6,
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,
        .acq_time   = NRF_SAADC_ACQTIME_40US,
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,
        .burst      = NRF_SAADC_BURST_ENABLED,
        .pin_p      = NRF_SAADC_INPUT_VDD,
        .pin_n      = NRF_SAADC_INPUT_DISABLED
    };

    nrf_saadc_resolution_set((nrf_saadc_resolution_t) 3);   // 3 is 14-bit
    nrf_saadc_oversample_set((nrf_saadc_oversample_t) 2);   // 2 is 4x, about 150uSecs total
    nrf_saadc_int_disable(NRF_SAADC_INT_ALL);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
    nrf_saadc_enable();

    NRF_SAADC->CH[1].CONFIG =
              ((myConfig.resistor_p << SAADC_CH_CONFIG_RESP_Pos)   & SAADC_CH_CONFIG_RESP_Msk)
            | ((myConfig.resistor_n << SAADC_CH_CONFIG_RESN_Pos)   & SAADC_CH_CONFIG_RESN_Msk)
            | ((myConfig.gain       << SAADC_CH_CONFIG_GAIN_Pos)   & SAADC_CH_CONFIG_GAIN_Msk)
            | ((myConfig.reference  << SAADC_CH_CONFIG_REFSEL_Pos) & SAADC_CH_CONFIG_REFSEL_Msk)
            | ((myConfig.acq_time   << SAADC_CH_CONFIG_TACQ_Pos)   & SAADC_CH_CONFIG_TACQ_Msk)
            | ((myConfig.mode       << SAADC_CH_CONFIG_MODE_Pos)   & SAADC_CH_CONFIG_MODE_Msk)
            | ((myConfig.burst      << SAADC_CH_CONFIG_BURST_Pos)  & SAADC_CH_CONFIG_BURST_Msk);

    NRF_SAADC->CH[1].PSELN = myConfig.pin_n;
    NRF_SAADC->CH[1].PSELP = myConfig.pin_p;
}

/**
 * @brief Function for 14-bit adc battery voltage by direct blocking reading
 */
int16_t GetBatteryVoltage1(void)
{			
		Adc12bitPolledInitialise();
		uint16_t result = 9999;         // Some recognisable dummy value
		uint32_t timeout = 10000;       // Trial and error
		volatile int16_t buffer[8];
		// Enable command
		nrf_saadc_enable();
		NRF_SAADC->RESULT.PTR = (uint32_t)buffer;
		NRF_SAADC->RESULT.MAXCNT = 1;
		nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
		nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
		nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);

		while (0 == nrf_saadc_event_check(NRF_SAADC_EVENT_END) && timeout > 0)
		{
			timeout--;
		}
		nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);
		nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
		nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
		// Disable command to reduce power consumption
		nrf_saadc_disable();
		if (timeout != 0)
		{		
			result = (((buffer[0] * 1000L)+(ADC12_COUNTS_PER_VOLT/2)) / ADC12_COUNTS_PER_VOLT);
			
			SEGGER_RTT_printf(0, "battery_voltage: %d\n", (result*4583)/100);
			
		}    
		return result;
}


/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
    {
        .adv_data =
            {
                .p_data = m_enc_advdata,
                .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX},
        .scan_rsp_data =
            {
                .p_data = NULL,
                .len = 0

            }};

static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] = /**< Information advertised by the Beacon. */
    {
        APP_BEACON_UUID
};

void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
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
            SEGGER_RTT_printf(0,"Connected \n");
            is_connect = true;
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
									
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            SEGGER_RTT_printf(0,"Disconnected \n");
            is_connect = false;
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            NVIC_SystemReset();	
            break;

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

        SEGGER_RTT_printf(0,"Received data from BLE NUS. \n");
        SEGGER_RTT_printf(0,p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

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
        if(p_evt->params.rx_data.length == 6){
	if(p_evt->params.rx_data.p_data[0] == 'L' && p_evt->params.rx_data.p_data[1] == 'e' && p_evt->params.rx_data.p_data[2] == 'd' && p_evt->params.rx_data.p_data[3] == 'O'){
	if(p_evt->params.rx_data.p_data[4] == 'n'){
						
	}else if(p_evt->params.rx_data.p_data[4] == 'f'){

						}
					}
			}	
    }

}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
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

static void advertising_init(void) {
      //GetBatteryVoltage1();
      uint32_t err_code;
      ble_advdata_t advdata;
      ble_advdata_manuf_data_t manuf_specific_data;

      manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
      get_gps_local(&m_beacon_info[0]);
      manuf_specific_data.data.p_data = (uint8_t *)m_beacon_info;
      manuf_specific_data.data.size = APP_BEACON_INFO_LENGTH;

      // Build and set advertising data.
      memset(&advdata, 0, sizeof(advdata));

      advdata.name_type = BLE_ADVDATA_NO_NAME;
      advdata.p_manuf_specific_data = &manuf_specific_data;

      // Initialize advertising parameters (used when starting advertising).
      memset(&m_adv_params, 0, sizeof(m_adv_params));

      m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
      m_adv_params.p_peer_addr = NULL; 
      m_adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
      m_adv_params.interval = APP_ADV_INTERVAL;
      m_adv_params.duration = 0;
       
      //raafat
      //counter++;
      //char *lonl=m_beacon_info[9];
      //printf("LONL is %s",latl);
      if(is_gps_fixed() ){
        pa_lna_setup();
      }
      //printf("\nAdv data %s \n",&m_beacon_info[9]);
      err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
      APP_ERROR_CHECK(err_code);

      err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
      APP_ERROR_CHECK(err_code);

      err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV,m_adv_handle,txpower);
      APP_ERROR_CHECK(err_code);


}

/**@brief Function for starting advertising.
 */
static void advertising_start(void) {
      ret_code_t err_code;

      err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
      APP_ERROR_CHECK(err_code);
}

static void advertising_stop(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_stop(m_adv_handle);
    APP_ERROR_CHECK(err_code);

}

static uint8_t cnt =0x30;
APP_TIMER_DEF(m_getdata_a_timer_id);
// Timeout handler for the repeated timer
static void timer_a_handler(void * p_context)
{
        int16_t VBAT = GetBatteryVoltage1();
 	if(is_connect){
          SEGGER_RTT_printf(0,"Connected Timer handler\n");
          cnt++;
          if(cnt > 0x39)
          cnt = 0x30;
          gps_test[63] = cnt;
          packet[55] = VBAT*5;               
          packet[63] = cnt;
          uint16_t length = 64;

          ble_nus_data_send(&m_nus, packet, &length,m_conn_handle);

	}else{
          SEGGER_RTT_printf(0,"DisConnected Timer handler\n");	}
      printf("\tTimer Handler\n");
      
      if(!is_gps_fixed() ){
        printf("Waiting GPS for initial fix\n");
        if (nrf_gpio_pin_read(7) == 0)
          {
              //enable GPS on 
              nrf_gpio_pin_clear(11);
              nrf_delay_ms(10);
              nrf_gpio_pin_set(11);
              nrf_delay_ms(50);
              nrf_gpio_pin_clear(11);
          }
////        advertising_init();
////        advertising_start();
////        nrf_delay_ms(200);
////        advertising_stop();
      }
      else{
        // try to toggle GPS mode
        nrf_gpio_pin_clear(11);
        nrf_delay_ms(10);
        nrf_gpio_pin_set(11);
        nrf_delay_ms(50);
        nrf_gpio_pin_clear(11);
      }
  ///enable GPS module
   if (nrf_gpio_pin_read(7) == 0)
    {
        printf("gps is Hibernate/Standby \n");
//      //GPS on 
//      nrf_gpio_pin_clear(11);
//      nrf_delay_ms(10);
//      nrf_gpio_pin_set(11);
//      nrf_delay_ms(50);
//      nrf_gpio_pin_clear(11);
//      printf("try reinit gps");
      //
//    advertising_init();
//    advertising_start();
//    nrf_delay_ms(10);
//    advertising_stop();

    }
    else{
      printf("gps is ON\n");
      //nrf_gpio_pin_set(13);
    }
    //advertising_stop();
    advertising_init();
    advertising_start();
    nrf_delay_ms(310);
    advertising_stop();
}
// Create timers
static void create_timers()
{  
    uint32_t err_code;

    // Create timers
    err_code = app_timer_create(&m_getdata_a_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_a_handler);
    APP_ERROR_CHECK(err_code);
}



static void ble_stack_init(void) {
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
}


/**@brief Function for initializing timers. */
static void timers_init(void) {
      ret_code_t err_code = app_timer_init();
      APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void) {
      ret_code_t err_code;
      err_code = nrf_pwr_mgmt_init();
      APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void) {
      nrf_pwr_mgmt_run();
}

#define NRF_CLOCK_LFCLKSRC            \
  { .source = NRF_CLOCK_LF_SRC_SYNTH, \
    .rc_ctiv = 0,                     \
    .rc_temp_ctiv = 0,                \
    .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM }




/**
 * @brief Function for application main entry.
 */
int main(void) {
 //gps_on_pin
    nrf_gpio_cfg_output(11);
    //nrf_gpio_cfg_output(13);
    //nrf_gpio_pin_clear(13);
    //raafat
   // nrf_gpio_pin_set(13);
    //nrf_delay_ms(1000);
    //nrf_gpio_pin_toggle(13);

    //nrf_delay_ms(400);
    //nrf_gpio_pin_clear(13);
    //nrf_gpio_cfg_output(18);
    //nrf_gpio_cfg_output(20);
   //nrf_gpio_pin_set(13);
    //nrf_gpio_pin_set(13);
    //nrf_gpio_pin_set(20);
    //nrf_gpio_pin_set(18);
    //GPS_wakeup read pin
    nrf_gpio_cfg_input(7,NRF_GPIO_PIN_PULLUP);
            
    if (nrf_gpio_pin_read(7) == 0)
    {
    //raafat
      //printf("try to turn on gps\n");
      //GPS on 
      nrf_gpio_pin_clear(11);
      nrf_delay_ms(10);
      nrf_gpio_pin_set(11);
      nrf_delay_ms(50);
      nrf_gpio_pin_clear(11);
      //
    }
    if (nrf_gpio_pin_read(7) == 1)
    {
      //printf("already wakeup");
      //Raafat
      //nrf_gpio_pin_clear(13);
    }
    if (nrf_gpio_pin_read(7) ==0)
    {
      //printf("GPS is OFF");
    }
    /////
      uint32_t err_code;
      timers_init();
      // Create application timers.
      create_timers();
      //Start timers
      err_code = app_timer_start(m_getdata_a_timer_id, APP_TIMER_TICKS(1000), NULL);
      APP_ERROR_CHECK(err_code);
      uart_init(GPS_UART_RX);
      power_management_init();
      ble_stack_init();
      gap_params_init();
      gatt_init();
      services_init();
      conn_params_init();
      //pa_lna_setup();
      advertising_init();

      // Start execution.
      advertising_start();
      // Enter main loop.
      for (;;) {
        idle_state_handle();
      }
}

/**
 * @}
 */