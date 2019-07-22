#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"
#include "nrf_drv_saadc.h"
#include "bsp.h"

#define APP_PA_LAN
#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */
#define DEVICE_NAME   			"BOTE"
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH          0x0C                              /**< Total length of information advertised by the Beacon. */
#define APP_DEVICE_TYPE                 0x02                               /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                               /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0xFFFF                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_BEACON_UUID                 0x59, 0x41, 0x48, 0x48, 0x49,\
																				0x00, 0x00, 0x00, 0x00, \
																				0x00, 0x00 /**< Proprietary UUID for Beacon. */

#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
// Battery level
// Input range of internal Vdd measurement = (0.6 V)/(1/6) = 3.6 V
// 3.0 volts -> 14486 ADC counts with 14-bit sampling: 4828.8 counts per volt
#define ADC12_COUNTS_PER_VOLT 4551
//uint16_t bat_volt;
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

///* TWI instance. */
//const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);
//
///**
// * @brief TWI initialization.
// */
//void twi_init (void)
//{
//	ret_code_t err_code;
//
//	const nrf_drv_twi_config_t twi_config = {
//		 .scl                = 7,
//		 .sda                = 6,
//		 .frequency          = NRF_DRV_TWI_FREQ_100K,
//		 .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
//		 .clear_bus_init     = false
//	};
//	// Nordic work around to prevent burning excess power due to chip errata
//	// Use 0x400043FFC for TWI0.
//	*(volatile uint32_t *)0x40004FFC = 0;
//	*(volatile uint32_t *)0x40004FFC;
//	*(volatile uint32_t *)0x40004FFC = 1;
//	///////
//	err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
//	APP_ERROR_CHECK(err_code);
//
//  nrf_drv_twi_enable(&m_twi);
//}
//void twi_uninit(void)
//{
//	nrf_drv_twi_disable( &m_twi );
//	nrf_drv_twi_uninit( &m_twi );
//}
///*
// * function to read sensor registers.
//*/
//ret_code_t read_register(nrf_drv_twi_t twi_instance, uint8_t device_addr, uint8_t register_addr, uint8_t *p_data, uint8_t bytes, bool no_stop)
//{
//  ret_code_t err_code;
//
//  err_code = nrf_drv_twi_tx(&twi_instance, device_addr, &register_addr, 1, no_stop);
//  APP_ERROR_CHECK(err_code);
//
//  if(err_code != NRF_SUCCESS) {
//    return err_code;
//  }
//
//  err_code = nrf_drv_twi_rx(&twi_instance, device_addr, p_data, bytes);
//  return err_code;
//}

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */

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


static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
	APP_BEACON_UUID,     // 128 bit UUID value.
	APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
											 // this implementation.
};


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
bool init=true;
static void advertising_init(void)
{
	uint32_t      err_code;
	ble_advdata_t advdata;
	uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

	ble_advdata_manuf_data_t manuf_specific_data;

	manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

	 // get sensor data
	uint8_t temp_val, temp_val_2, humid_val, humid_val_2;
//	Adc12bitPolledInitialise();
//	GetBatteryVoltage1();
//	nrf_drv_saadc_abort();
//	nrf_drv_saadc_uninit();
	m_beacon_info[9] = bat_volt1;
	m_beacon_info[10] = bat_volt2;
	
	manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
	manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

	// Build and set advertising data.
	memset(&advdata, 0, sizeof(advdata));

	advdata.name_type             = BLE_ADVDATA_NO_NAME;
	advdata.flags                 = flags;
	advdata.p_manuf_specific_data = &manuf_specific_data;

	// Initialize advertising parameters (used when starting advertising).
	memset(&m_adv_params, 0, sizeof(m_adv_params));

	m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
	m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
	m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
	m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
	m_adv_params.duration        = 0;       // Never time out.

	err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
	APP_ERROR_CHECK(err_code);

	err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
	APP_ERROR_CHECK(err_code);
	///Set device name
	ble_gap_conn_sec_mode_t sec_mode;
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
	err_code = sd_ble_gap_device_name_set(&sec_mode,(const uint8_t *)DEVICE_NAME,strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);
	////
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
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

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
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
}


APP_TIMER_DEF(m_getdata_a_timer_id);
// Timeout handler for the repeated timer
static void timer_a_handler(void * p_context)
{
	advertising_stop();
	advertising_init();
	advertising_start();
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


/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

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
  nrf_pwr_mgmt_run();
}


#ifdef APP_PA_LAN
#define APP_PA_PIN              	17
#define APP_LNA_PIN              	19


#define APP_CPS_PIN			6

#define APP_AMP_PPI_CH_ID_SET   0
#define APP_AMP_PPI_CH_ID_CLR   1
#define APP_AMP_GPIOTE_CH_ID    0

static void pa_lna_setup(void)
{
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
                    .gpio_pin = APP_PA_PIN
                },
							
								
                .lna_cfg = {
                    .enable = 1,
                    .active_high = 1,
                    .gpio_pin = APP_LNA_PIN
                },
								
                .ppi_ch_id_set = APP_AMP_PPI_CH_ID_SET,
                .ppi_ch_id_clr = APP_AMP_PPI_CH_ID_CLR,
                .gpiote_ch_id = APP_AMP_GPIOTE_CH_ID
            }
        }
    };
    NRF_GPIO->DIRSET |= (1 << APP_PA_PIN) | (1 << APP_LNA_PIN) ;
    err_code = sd_ble_opt_set(BLE_COMMON_OPT_PA_LNA, &pa_lna_opts);
    APP_ERROR_CHECK(err_code);

}
#endif

/**
 * @brief Function for application main entry.
 */
int main(void)
{
// Initialize.
    uint32_t err_code;
    timers_init();
    // Create application timers.
    create_timers();
    //Start timers
    err_code = app_timer_start(m_getdata_a_timer_id, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);
    power_management_init();
    pa_lna_setup();
    ble_stack_init();
    advertising_init();
    advertising_start();
    // Enter main loop.
    for (;; )
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
