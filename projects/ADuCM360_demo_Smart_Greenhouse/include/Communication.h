#ifndef _COMMUNICATION_H_
   #define _COMMUNICATION_H_

   /* UART Definitions */
   #define DEMO_BAUD_RATE     B115200
   #define DEMO_BIT_SIZE      COMLCR_WLS_8BITS

   #define _CR                13
   #define _LF                10
   #define _SPC               32
   #define _BS                8

   #define UART_RX_BUFFER_SIZE   256
   #define UART_TRUE             1
   #define UART_FALSE            0
   #define UART_SUCCESS          0

   /* SPI Definitions */
   #define CN0397_CS_PORT  pADI_GP0
   #define CN0397_CS_PIN   PIN3
   #define CN0397_CS_BIT   BIT3

   #define CN0398_CS_PORT  pADI_GP0
   #define CN0398_CS_PIN   PIN4
   #define CN0398_CS_BIT   BIT4

   #define RED_LED_CS_PORT    pADI_GP1
   #define RED_LED_CS_PIN     PIN0
   #define RED_LED_CS_BIT     BIT0

   #define BLE_LED_CS_PORT    pADI_GP1
   #define BLE_LED_CS_PIN     PIN1
   #define BLE_LED_CS_BIT     BIT1

   #define GRN_LED_CS_PORT    pADI_GP2
   #define GRN_LED_CS_PIN     PIN2
   #define GRN_LED_CS_BIT     BIT2

   #define RED_LED            0
   #define BLE_LED            2
   #define GRN_LED            1

   #define CN0397_SLAVE       0
   #define CN0398_SLAVE       1

   #define ADP7118_PORT       pADI_GP1
   #define ADP7118_BIT        0x04
   #define ADP7118_PIN        PIN2

   /* I2C Definitions */
   #define I2C_PRINT_RAW      1

   extern void Comms_Init(void);

   extern void UART_Init(long lBaudrate, int iBits);
   extern void UART_WriteChar(char c);
   extern char UART_ReadChar(void);
   extern int _write (int fd, char *ptr, int len);

   extern void SPI_Init(void);
   extern void SPI1_Disable(void);
   extern void SPI1_Enable(void);
   extern void SPI0_Write(uint16_t data, unsigned char channel);
   extern void SPI1_Write(unsigned char slaveDeviceId, unsigned char *data, unsigned char bytesNumber);
   extern void SPI1_Read(unsigned char slaveDeviceId, unsigned char* data, unsigned char bytesNumber);

   extern void SPI_Write(unsigned char* data, unsigned char bytesNumber);
   extern void SPI_Read(unsigned char* data, unsigned char bytesNumber);

   extern void I2C_Init(void);
   extern uint8_t I2C_Write(uint8_t *ui8Data, uint8_t NumBytes, uint8_t address);
   extern uint8_t I2C_Read(uint8_t NumBytes, uint8_t address);

   extern uint8_t spi1TxComplete;
   extern uint8_t spi1RxComplete;

   extern uint8_t spi0TxComplete;
   extern uint8_t convFlag;

   extern uint8_t rxI2C;
   extern uint8_t txI2C;

   extern uint8_t rxI2Csize;
   extern uint8_t txI2Csize;

   extern uint8_t *rxI2Cbuf;
   extern uint8_t *txI2Cbuf;

   extern uint8_t rxI2Ccomplete;
   extern uint8_t txI2Ccomplete;

   extern uint8_t uart_rcnt;
   extern uint8_t uart_cmd;
   extern uint8_t uart_rdy;
   extern uint8_t uart_read_ch;
   extern char uart_rx_char;
   extern unsigned char uart_rx_buffer[UART_RX_BUFFER_SIZE];

#endif
