/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
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
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"
static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");
    printf("transfer");
    if (m_rx_buf[0] != 0)
    {
        NRF_LOG_INFO(" Received:");
        printf("Received");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}


//write(20h)
//write(17h)

/*

WRITE = 0
READ = 1

write (add, data){

CS
send_byte( WRITE + add);
send_byte(data);
CS


}


*/

int main(void)
{
    //BOTE not needed
    
    //bsp_board_init(BSP_INIT_LEDS);
    // GPS
    #define GPS_ONOFF_PIN   23    //for GPS
    #define GPS_RESET_PIN   16    //for GPS
    #define GPS_WAKEUP_PIN  7     //for GPS

    nrf_gpio_cfg_input(GPS_WAKEUP_PIN,NRF_GPIO_PIN_PULLUP);

    //nrf_gpio_cfg_input(GPS_RESET_PIN,0);   // IMPORTANT  RESET must NOT be HIGH   Do not drive this input high!!!
    nrf_delay_ms(10);
    nrf_gpio_cfg_output(GPS_ONOFF_PIN);   //for GPS
    nrf_gpio_pin_clear(GPS_ONOFF_PIN);  //for GPS
    nrf_delay_ms(1000);




    // RESET input is active low and has internal pull-up resistor of 86k?to internal 1.2V domain
    
    //nrf_gpio_cfg_input(SPI_MISO_PIN);


//    nrf_gpio_pin_set(SPI_MISO_PIN);
//    nrf_gpio_pin_set(SPI_MISO_PIN);
//    nrf_gpio_pin_set(SPI_MOSI_PIN);
//    nrf_gpio_pin_set(SPI_SS_PIN);
    //nrf_gpio_pin_clear(SPI_SCK_PIN);   // CTS must be LOW

    //  for ACCELEROMETER
    //nrf_gpio_cfg_output(SPI_SS_PIN); 
    //nrf_gpio_pin_clear(SPI_SS_PIN);

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;

    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

    printf("Transfer test completed.");
//    NRF_LOG_INFO("SPI example started.");
//    NRF_LOG_INFO("Transfer test completed.");

    while (1)
    {

    nrf_delay_ms(100);

    nrf_gpio_pin_set(GPS_ONOFF_PIN);  //for GPS
    nrf_delay_ms(1);
    nrf_gpio_pin_clear(GPS_ONOFF_PIN);  //for GPS
    nrf_delay_ms(100);

    if(nrf_gpio_pin_read(GPS_WAKEUP_PIN) == 0)
      {
        printf("GPS is SLEEPING.");
      }else{
        printf("GPS is AWAKE");
      }

        // Reset rx buffer and transfer done flag
        memset(m_rx_buf, 0, m_length);
        spi_xfer_done = false;

        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));

        while (!spi_xfer_done)
        {
            __WFE();
            printf("print is working");
        }

        NRF_LOG_FLUSH();

        //bsp_board_led_invert(BSP_BOARD_LED_0);
        nrf_delay_ms(200);
    }
}
