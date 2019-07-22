#include <string.h>
#include "nmea.h"

unsigned char hex2bin( char c )
{
  if( ( c >= 'A') && ( c <= 'F') ) {
    return c - 'A';    
  }   
  else if( ( c >= 'a') && ( c <= 'f') ) {
    return c - 'a';    
  }
  else {
    return c - '0';    
  }
}


char nmea_check_crc( char *buf )
{
  unsigned char crc = 0;
  while( *buf != '*' ) {
     crc ^= *buf++;
  }
  
  return ( ( crc & 0x0F ) == hex2bin( *buf ) ) && 
         ( ( crc >> 4   ) == hex2bin( *(buf+1) ) );  
}

#define ARRAY_SIZE( array )  (sizeof(array)/sizeof(array[0]))



//,  nmea 
typedef struct _nmea_message {
  char * id; // 
  char (*parser)( char *buf, void *data ); // 
  void *data; //  
} nmea_message_t;



char *nmea_next_field( char* buf)
{
    // return strtok( buf, ',' );
    while( *buf++ != ',' );
      return buf;
}

typedef struct gms_rmc {
  double utc_time;     //время в 0-ой зоне
  unsigned char valid; //признак достоверности данных
  double latitude;     //широта
  double longitude;    //долгота
  double speed;        //скорость в милях
  double direction;    //направление движения 
  unsigned long data;  //дата
} nmea_rmc_t;
 
//nmea_rmc_t nmea_rmc_data;
//typedef struct gms_rmc {
//  unsigned short valid;
//} nmea_rmc_t;

nmea_rmc_t nmea_rmc_data;

//$GPRMC,171729.000,A,6001.5972,N,03012.7352,E,0.03,0.00,111212,,,A*6B
char nmea_rmc_parser( char *buf, void *data )
{
  char *ptr_end;
  nmea_rmc_t* rmc = (nmea_rmc_t*)data;
  memset( rmc, 0, sizeof( nmea_rmc_t ) );


  printf( "buf =%s" , buf );
  rmc->utc_time = strtod( buf, &ptr_end );
  printf( "time %lu \r\n", (unsigned long)rmc->utc_time  );
  buf = nmea_next_field( buf );


  if( *buf == 'A') {
  }
  buf = nmea_next_field( buf );

  printf( "buf =%s" , buf );
  rmc->latitude = atof( buf );
  buf = nmea_next_field( buf );

  if( *buf == 'S') {
   rmc->latitude = -rmc->latitude;
  }
  printf( "latitude %lu \r\n", (unsigned long)rmc->latitude );
  buf = nmea_next_field( buf );

  printf( "buf =%s" , buf );
  rmc->longitude = atof( buf );
  buf = nmea_next_field( buf );
  if( *buf == 'W') {
   rmc->longitude = -rmc->longitude;
  }
  printf( "longitude %lu \r\n", (unsigned long)rmc->longitude );


  buf = nmea_next_field( buf );
  printf( "buf =%s" , buf );
//  rmc->fix_status  = atoi( buf );
//  printf( "fix_status %u \r\n", rmc->fix_status );


  buf = nmea_next_field( buf );
  printf( "buf =%s" , buf );
//  rmc->satelits = atoi( buf );
//
//  printf( "satelits %u \r\n", rmc->satelits );

  
  return 0;
}



nmea_message_t nmea_messages[] =
{
/*
   {
   .cmd = "GPGGA",
   .parser = nmea_gga_parser,
   .data = &nmea_gga_data,
   },
 */
  {
    .id = "GPRMS",
    .parser = nmea_rmc_parser,
    .data = &nmea_rmc_data,
  },
};




unsigned char nmea_parser( char* buf )
{
  unsigned char i;
  
  if( *buf++ != '$' ) {
    return -1;
  }

  if( !nmea_check_crc( buf )  ) {
    return -1; //CRC fail
  }

  for( i = 0; i< ARRAY_SIZE( nmea_messages ); i++ ) {
    if( strncmp( buf, nmea_messages[i].id, strlen( nmea_messages[i].id ) ) == 0 ) {
      if( nmea_messages[i].parser ) {
        buf = nmea_next_field( buf );
        return nmea_messages[i].parser( buf, nmea_messages[i].data );
      }
      return -1;
    }
  }

  return -1;
}
