
#include <string.h>

#include "app_uart.h"
#include "boards.h"

#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

static char gps_local[128];

typedef enum 
{
    ID, 
    GPGGA, 
    MESSAGE
} gps_index_t;

#define NR_OF_GPS_DATA 8

gps_index_t gps_index = ID;
char bufferIn[100];
uint8_t counter = 0;
uint8_t gps_msg_index = 0;
bool gps_updated = false;

void get_gps_local(char *gps_out)
{
    strcpy(gps_out, gps_local);
}

bool is_gps_updated()
{
    bool ret_val = gps_updated;
    if(gps_updated)
    {
        gps_updated = false;
    }
    return ret_val;
}

void gps_handler(uint8_t c)
{
    //SEGGER_RTT_printf(0,"String from gps %s \n", gps_index );
    //app_uart_put(c);
    switch(gps_index){
        case ID:
            if(c == '$')
            {
                gps_index = GPGGA;
            }
            break;
        case GPGGA:
            if (c == ',')
            {
                if (strcmp(bufferIn, "GPGGA") == 0)  //if (strcmp(bufferIn, "GPGGA") == 0)
                {
                SEGGER_RTT_printf(0,"String from gps 1 %s ", bufferIn );
                    gps_index = MESSAGE;
                }
                else
                {
                    gps_index = ID;
                }
                memset(bufferIn, '\0', sizeof(bufferIn));
                counter = 0;
            } 
            else
            {
                bufferIn[counter] = c;
                counter++;
            }
            break;
        case MESSAGE:
            if (c == ',')
            {

                gps_msg_index++;
                if (gps_msg_index > NR_OF_GPS_DATA)
                {
                    gps_index = ID;
                    gps_msg_index = 0;
                    counter = 0;
                    SEGGER_RTT_printf(0,"%s \n", bufferIn );
                    strcpy(gps_local, bufferIn);
                    gps_updated = true;
                    
                    memset(bufferIn, '\0', sizeof(bufferIn));
                }
                else
                {
                    bufferIn[counter] = c;
                    counter++;
                }
            }
            else
            {
                bufferIn[counter] = c;
                counter++;
            }
            break;
    }   
}

void uart_error_handle(app_uart_evt_t * p_event)
{
    if(p_event->evt_type == APP_UART_DATA_READY)
    {
        uint8_t data;
        app_uart_get(&data);
        //SEGGER_RTT_printf(0,"String from UART %s \n", data );
        gps_handler(data);
    }
    else if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        //APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        //APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

void uart_init(uint8_t rx_pin_number)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
      {
          rx_pin_number,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud4800
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);
}
