#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

#include <ADuCM360.h>
#include <SpiLib.h>
#include <DioLib.h>

#include <stddef.h>

#ifdef  __cplusplus
extern "C" {
#endif
/********************************* Internal defines ****************************/
#define UART_TX_BUFFER_SIZE      1024       // UART transmit buffer size
#define UART_RX_BUFFER_SIZE      256        // UART receive buffer size

/* Execution status */
#define UART_SUCCESS             0
#define UART_FAILURE            -1
#define UART_NO_TX_SPACE        -2
#define UART_NO_RX_SPACE        -3

/* UART status */
#define UART_TRUE                1
#define UART_FALSE               0

/* CS_AD7124 - 0.3 - output */
#define CS_PORT      pADI_GP0
#define CS_PIN       0x08
#define CS_PIN_NUMBER   PIN3

/* ADP7118 - 1.2 - output */
#define ADP7118_PORT      pADI_GP1
#define ADP7118_PIN       0x04
#define ADP7118_PIN_NUMBER   PIN2

extern uint8_t convFlag;

extern char Rx_char;
extern bool read_ch;

extern unsigned char          uart_rx_buffer[UART_RX_BUFFER_SIZE];
extern unsigned char          uart_tx_buffer[UART_TX_BUFFER_SIZE];

/****************************** Internal types *********************************/


/* Write data mode */
typedef enum {
   UART_WRITE_NO_INT = 1,            /* Write data when interrupts are disabled */
   UART_WRITE_IN_INT,               /* Write data while in an interrupt routine */
   UART_WRITE
} enWriteData;


extern unsigned char ucTxBufferEmpty;       // Used to indicate that the UART Tx buffer is empty
extern unsigned char ucWaitForUart;          // Used by calibration routines to wait for user input


#ifdef  __cplusplus
}
#endif // __cplusplus

class SPIClass {
public:

  static void Init();
  static void Write(unsigned char slaveDeviceId, unsigned char* data, unsigned char bytesNumber);
  static void Read(unsigned char slaveDeviceId, unsigned char* data, unsigned char bytesNumber);

};

class UARTClass {
public:

  static void Init();
  static void WriteChar(char c);
  char ReadChar(void);

};


extern SPIClass SPI;
extern UARTClass UART;

#endif
