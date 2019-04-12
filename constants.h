#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <RFM69X.h>

//*********************************
//**** IMPORTANT RADIO SETTINGS****
//*********************************
#define GATEWAYID       1 
#define NODEID          2
#define NETWORKID       100
#define FREQUENCY       RF69_915MHZ
#define ENCRYPTKEY      "sampleEncryptKey" //has to be same 16 bytes
#define IS_RFM69HW      true //Only for RFM69HW/HCW

#define TRANSMITPERIOD      200
#define SEND_PROMISCUOUS    false
#define SEND_RETRY          0
#define SEND_WAIT_WDG       40

#define SERIAL_BAUD     115200

#define LED             2
#define RFM_SS          SS
#define RFM_INT         15

byte sendSize = 0;
long int frameCount = 0;
long int ackSentCount = 0;

//********************************************
//**** PARA OBTENER LA FECHA Y HORA ACTUAL****
//********************************************
#define TZ_INFO "CST6CDT5,M4.1.0/02:00:00,M10.5.0/02:00:00"
#define timeInt32 1554487155

//********************************************
//**** PARAMETROS PARA DEEPSLEEP ****
//********************************************
#define TIME_TO_SLEEP  5
#define uS_TO_S_FACTOR 1000000


//********************************************
//**** PARAMETROS PARA DATALOGGER ****
//********************************************
#define EEPROM_SIZE     0x2000

//********************************************
//**** PARAMETROS PARA ENCODER ****
//********************************************
#define PINA    1
#define PINB    2
#endif
