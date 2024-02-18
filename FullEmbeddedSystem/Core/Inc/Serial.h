// Serial.h


#ifndef _SERIAL_H_
#define _SERIAL_H_


// Includes
#include "main.h"
#include <stdint.h>


// typedefs, structs, and enums
typedef enum _strip_zeros_ {
  StripZeros,
  NoStripZeros
} strip_zeros;

typedef enum _add_crlf_ {
  AddCRLF,
  NoAddCRLF
} add_CRLF;

// defines
#define TX_BUFFER_SIZE 50
#define RX_BUFFER_SIZE 50



// private modular variables


// public variables
extern uint8_t rxBuffer[];
extern uint8_t txBuffer[];
extern uint8_t nextSerialTxOut;
extern uint8_t nextSerialTxIn;
extern uint8_t nextSerialRxIn;
extern uint8_t nextserialRx2Proc;

// public functions prototypes
uint8_t SendString(const char * _msg, uint16_t _len, strip_zeros _supressZeros, add_CRLF _add_crlf);
uint16_t CheckBuffer(void);


#endif
