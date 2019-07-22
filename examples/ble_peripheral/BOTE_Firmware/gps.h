#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <stdbool.h>

extern char *UTC,*lat,*latl,*lon,*lonl, *valid;
void uart_init(uint8_t rx_pin_number);
void get_gps_local(char *gps_out);
bool is_gps_updated(void);
bool is_gps_fixed(void);
bool turn_Off_GPS();
bool turn_ON_GPS();
bool toggle_GPS();
#endif //GPS_H
