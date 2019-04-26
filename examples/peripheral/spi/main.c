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

/* Bote GPS pins:
 *
 GPS POWER ON               P0.23
 GPS RESET                  P0.16

 TX - SPI data out = MISO   P0.04
 RX - SPI data in = MOSI    P0.05
 RTS - chip select = CS     P0.10
 CTS - SPI clock = SCK      P0.09

 GPS wakeup                 P0.07
 *
 */
#define GPS_ONOFF_PIN   23
#define GPS_RESET_PIN   16
#define GPS_WAKEUP_PIN   7

#define GPS_NMEA_MESSAGE_MAX_SIZE 120

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"
static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[GPS_NMEA_MESSAGE_MAX_SIZE];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

/**
 * @brief SPI user event handler.
 * @param event
 */
void gps_spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");
    if (m_rx_buf[0] != 0)
    {
        NRF_LOG_INFO(" Received: first char %c", m_rx_buf[0]);
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));

        // idle pattern of ORG1411 GPS is ‘0xA7 0xB4’
        // The first message to come out of module is “OK_TO_SEND” - ‘$PSRF150,1*3E’.

        // We should get $GPGGA GPS position message every 1 second by default.
        // GPS NMEA format:
        // https://www.gpsworld.com/what-exactly-is-gps-nmea-data/
    }
}

void gps_spi_init(void)
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;

    spi_config.mode     = NRF_SPI_MODE_1; // SCK active high, sample on trailing edge of clock.
    spi_config.frequency = SPI_FREQUENCY_FREQUENCY_M4;
    // only SS/CS active low supported by SPIM driver. Otherwise, needs to be controlled manually.

    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, gps_spi_event_handler, NULL));
    NRF_LOG_INFO("SPI init done.");
}

// TODO: move GPS function to separate module
bool gps_is_cold_start(void)
{
    // We will save this info to nRF52 flash when going to sleep.
    // For now, always do cold start
    bool cold_start = true;
    return cold_start;
}

void gps_pulse(uint32_t ms_time)
{
    nrf_gpio_pin_set(GPS_ONOFF_PIN);
    nrf_delay_ms(ms_time);
    nrf_gpio_pin_clear(GPS_ONOFF_PIN);
}

void gps_init(void)
{
    gps_spi_init();

    // Power on GPS. Send ON_OFF pulse to go from hibernate to Full Power.
    nrf_gpio_cfg_output(GPS_ONOFF_PIN);
    gps_pulse(100);

    // Leave RESET in High Z.
    
    NRF_LOG_INFO("GPS power init done.");
    // TODO: SPI driver should pull down CS?

    // Cold start - after first power on wait 35 seconds for the first valid output
    if (gps_is_cold_start)
    {
        NRF_LOG_INFO("Cold start, wait for wake-up pin before getting first data. May take up to 35 seconds...");
        
        nrf_gpio_cfg_input(GPS_WAKEUP_PIN, GPIO_PIN_CNF_PULL_Disabled);
        uint32_t wakeup_state = 0;

        while (!wakeup_state)
        {
            wakeup_state = nrf_gpio_pin_read(GPS_WAKEUP_PIN);
            NRF_LOG_INFO("GPS ready? Wake-up pin: %d", wakeup_state);
            
            if (!wakeup_state)
            {
                gps_pulse(100);
            }

            NRF_LOG_FLUSH();
            nrf_delay_ms(1000);
        }


        // TODO: set interrupt on Wake-up pin to check if GPS is in Full Power state (it can be ready earlier than 35 seconds)
        // Host must initiate SPI connection approximately 1s after WAKEUP output goes high.
        nrf_delay_ms(1000);
        NRF_LOG_INFO("GPS ready!");
    }
}


int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO("Logger initialized.");
    NRF_LOG_FLUSH();

    gps_init();

    while (1)
    {
        // Reset rx buffer and transfer done flag
        memset(m_rx_buf, 0, sizeof(m_rx_buf));
        spi_xfer_done = false;

        NRF_LOG_INFO("SPI transfer");
        NRF_LOG_FLUSH();
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, sizeof(m_tx_buf), m_rx_buf, sizeof(m_rx_buf)));

        while (!spi_xfer_done)
        {
            __WFE();
        }

        NRF_LOG_FLUSH();
        nrf_delay_us(10000); // not really needed, remove to speed up communication
    }
}
