
#include <string.h>
#include <stdbool.h>
#include "app_uart.h"
#include "boards.h"
#include "nmea.h"
//raafat
#include "nrf_delay.h"
#include "SEGGER_RTT.h"
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

static char gps_local[128];
bool gps_fixed=false;
typedef enum 
{
    ID, 
    GPGGA, 
    MESSAGE
} gps_index_t;

gps_index_t gps_index = ID;
char bufferIn[100];
uint8_t counter = 0;
bool gps_updated = false;
//extern is_gps_fixed=false;
//
void get_gps_local(char *gps_out)
{
    strcpy(gps_out, gps_local);
}



///
char *UTC,*lat,*latl,*lon,*lonl, *valid;
double latitude;
void get_gps_test(char *gps_out)
{
  printf(gps_out);
  char *tok = strtok(gps_out, ",");  //begins isolating fields of information delimited by a comma
  int comma=0;
  while (tok != NULL) 
  {
    comma++;
    switch (comma) 
    {
    case 1: UTC=tok; break;   //GMT time in seconds, saves pointer to string
    case 2: lat=tok; break;   //latitude
    case 3: latl=tok; break;   //lat letter, (N or S)
    case 4: lon=tok; break;   //longitude
    case 5: lonl=tok; break;   //lon letter (W or E)
    case 6: valid=tok; break;   //valid
    }
    tok = strtok(NULL,",");
    
  } 
  latitude = atof(lat);
  sprintf(gps_local, "%s,%s,%s,%s",lat,latl,lon,lonl);
  if (latl=="N" || latl=="S") { gps_fixed=true;}
    else{gps_fixed=false;}
  //printf("Latitude  value is %s \n",lat);
  SEGGER_RTT_printf(0,"GPS DATA = %s time = %s\n", gps_local, UTC);
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

bool is_gps_fixed()
{
  return gps_fixed;
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
                //SEGGER_RTT_printf("first letter of message");
            }
            break;
        case GPGGA:
            if (c == ',')
            {
                if (strcmp(bufferIn, "GPGGA") == 0)  //if (strcmp(bufferIn, "GPGGA") == 0)
                {
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
            if (c == '\n')
            {
               gps_index = ID;
               counter = 0;
               get_gps_test(bufferIn);
               //strcpy(gps_local, bufferIn);
               gps_updated = true;
               memset(bufferIn, '\0', sizeof(bufferIn));
            }
            else
            {
                bufferIn[counter] = c;
                counter++;
                //printf("counter %s \n"+counter);
            }
            break;
    }   
}

void uart_error_handle(app_uart_evt_t * p_event)
{
    //SEGGER_RTT_printf(0,"UART error handler\n");
    //printf(".");
    if(p_event->evt_type == APP_UART_DATA_READY)
    {
        uint8_t data;
        app_uart_get(&data);
        SEGGER_RTT_printf(0,"String from UART %s \n", data );
        gps_handler(data);
        
    }
    else if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        SEGGER_RTT_printf(0,"APP_UART_COMMUNICATION_ERROR\n" );

        //APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        SEGGER_RTT_printf(0,"APP_UART_FIFO_ERROR \n" );
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
          //UART_BAUDRATE_BAUDRATE_Baud9600
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);
}

bool turn_Off_GPS(){
    nrf_gpio_cfg_input(7,NRF_GPIO_PIN_PULLUP);	
    if (nrf_gpio_pin_read(7) == 1)
    {
      //GPS pulse 
      nrf_gpio_pin_clear(11);
      nrf_delay_ms(10);
      nrf_gpio_pin_set(11);
      nrf_delay_ms(50);
      nrf_gpio_pin_clear(11);
      //
    }
   return true;
}

bool turn_ON_GPS(){
    nrf_gpio_cfg_input(7,NRF_GPIO_PIN_PULLUP);	
    nrf_gpio_cfg_output(11);
    if (nrf_gpio_pin_read(7) == 0)
    {
      //GPS on 
      nrf_gpio_pin_clear(11);
      nrf_delay_ms(10);
      nrf_gpio_pin_set(11);
      nrf_delay_ms(50);
      nrf_gpio_pin_clear(11);
      
    }
   return true;
}

// return TRUE if GPS turns ON and FALSE if GPS turns off
bool toggle_GPS(){
    nrf_gpio_cfg_input(7,NRF_GPIO_PIN_PULLUP);	
      //GPS on 
      nrf_gpio_pin_clear(11);
      nrf_delay_ms(10);
      nrf_gpio_pin_set(11);
      nrf_delay_ms(50);
      nrf_gpio_pin_clear(11);
      //

     if (nrf_gpio_pin_read(7) == 1)  return true; else return false;
    
}

