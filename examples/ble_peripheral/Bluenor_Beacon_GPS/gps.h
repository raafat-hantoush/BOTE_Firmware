#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <stdbool.h>

void uart_init(uint8_t rx_pin_number);
void get_gps_local(char *gps_out);
bool is_gps_updated(void);

#endif //GPS_H
