#include "app_timer.h"
#include "ble_advdata.h"
#include "bsp.h"
#include "nordic_common.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_soc.h"
#include <stdbool.h>
#include <stdint.h>
#include "nrf_drv_saadc.h"
#include "gps.h"
#include "app_uart.h"
#include "nrf_delay.h"

#define GPS_UART_RX               4
    //TX powers accepted values are -40, -30, -20, -16, -12, -8, -4, 0, and 4 dBm
int txpower =+4;
//raafat
bool is_advertising=false;
static int counter=0;
#define APP_BLE_CONN_CFG_TAG      1 /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL MSEC_TO_UNITS(150, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH    0x1B   /**< Total length of information advertised by the Beacon. */

#define APP_ADV_DATA_LENGTH       0x1B      /**< Length of manufacturer specific data in the advertisement. */

#define APP_COMPANY_IDENTIFIER    0xFFFF /**< Company identifier as per www.bluetooth.org. */

#define APP_BEACON_UUID           0x59, 0x41, 0x48, 0x48, 0x49  /**< Proprietary UUID for Beacon. */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static ble_gap_adv_params_t m_adv_params;                     /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */

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

  NRF_GPIO->DIRSET |= (1 << APP_PA_PIN) | (1 << APP_LNA_PIN) ;
  //nrf_gpio_pin_set(APP_PA_PIN);
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
void GetBatteryVoltage1(void)
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
    }
    bat_volt1=result >> 8;
    bat_volt2=result & 0xFF;	
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


static void advertising_init(void) {
      GetBatteryVoltage1();
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
      m_adv_params.interval = NON_CONNECTABLE_ADV_INTERVAL;
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
      //raafat
      //  here the PA is enabled for advertising and thn turned off.
      //nrf_gpio_pin_set(APP_PA_PIN);
      err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
      APP_ERROR_CHECK(err_code);
      is_advertising=true;
      //raafat 
      //nrf_gpio_pin_clear(APP_PA_PIN);
      printf("\t Advertising started\n");
      //  ///enable GPS module
      //   if (nrf_gpio_pin_read(7) == 0)
      //    {
      //      //GPS on 
      //      nrf_gpio_pin_clear(11);
      //      nrf_delay_ms(10);
      //      nrf_gpio_pin_set(11);
      //      nrf_delay_ms(50);
      //      nrf_gpio_pin_clear(11);
      //      printf("try reinit gps");
      //      //
      //    }
      //    else{
      //    //raafat
      //      printf("gps is ON\n");
      //      //nrf_gpio_pin_set(13);

      //    }
      
}

static void advertising_stop(void)
{
    ret_code_t err_code;
    if (is_advertising){
      err_code = sd_ble_gap_adv_stop(m_adv_handle);
      APP_ERROR_CHECK(err_code);
      is_advertising=false;
      printf("\t advertising stopped\n");
    }
}

APP_TIMER_DEF(m_getdata_a_timer_id);
// Timeout handler for the repeated timer
static void timer_a_handler(void * p_context)
{
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
    //nrf_gp5io_pin_set(13);

    //GPS_wakeup read pin
    nrf_gpio_cfg_input(7,NRF_GPIO_PIN_PULLUP);
            
    if (nrf_gpio_pin_read(7) == 0)
    {
    //raafat
      printf("try to turn on gps\n");
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
      printf("already wakeup\n");
      //Raafat
      //nrf_gpio_pin_clear(13);
    }
    if (nrf_gpio_pin_read(7) ==0)
    {
      printf("GPS is OFF\n");
    }
    
      uint32_t err_code;
      timers_init();

      uart_init(GPS_UART_RX);
      power_management_init();
      ble_stack_init();
      //pa_lna_setup();
      sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
      //advertising_init();
      //nrf_delay_ms(5000);
      // Start execution.
      //advertising_start();

      // Raafat
      // Create application timers.
      create_timers();
      //Start timers
      err_code = app_timer_start(m_getdata_a_timer_id, APP_TIMER_TICKS(2000), NULL);
      APP_ERROR_CHECK(err_code);
      // Start execution.
      SEGGER_RTT_printf(0,"APP started.\n");
      // Enter main loop.
      for (;;) {
        idle_state_handle();
      }
}

/**
 * @}
 */