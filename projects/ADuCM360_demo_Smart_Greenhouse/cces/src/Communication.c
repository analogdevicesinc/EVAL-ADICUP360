/***************************** Library Include Files **************************/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/***************************** Source Include Files ***************************/
#include "ADuCM360.h"
#include "UrtLib.h"
#include "SpiLib.h"
#include "I2cLib.h"
#include "DioLib.h"
#include "IntLib.h"
#include "Timer.h"
#include "Communication.h"

/***************************** Class Variables ********************************/
/* SPI Variables */
uint8_t spi1TxComplete = 0;
uint8_t spi1RxComplete = 0;
uint8_t spi0TxComplete = 0;
uint8_t convFlag = 0;

/* I2C Variables */
uint8_t rxI2C = 0;
uint8_t txI2C = 0;

uint8_t rxI2Csize = 0;
uint8_t txI2Csize = 0;

uint8_t *rxI2Cbuf;
uint8_t *txI2Cbuf;

uint8_t rxI2Ccomplete = 0;
uint8_t txI2Ccomplete = 0;

/* UART Variables */
uint8_t uart_rcnt, uart_cmd, uart_rdy = 0, uart_read_ch;
char uart_rx_char;
unsigned char uart_rx_buffer[UART_RX_BUFFER_SIZE];

void Comms_Init(void)
{
   //I2C_Init();
   UART_Init(DEMO_BAUD_RATE, DEMO_BIT_SIZE);
   SPI_Init();
   //NVIC_EnableIRQ(I2CM_IRQn);
   //NVIC_EnableIRQ(SPI1_IRQn);
   //NVIC_EnableIRQ(SPI0_IRQn);
   NVIC_EnableIRQ(UART_IRQn);
}

void UART_Init(long lBaudrate, int iBits)
{
   DioCfgPin(pADI_GP0, PIN6, 1);
   DioCfgPin(pADI_GP0, PIN7, 2);

   UrtCfg(pADI_UART, lBaudrate, iBits, 0);
   UrtMod(pADI_UART, COMMCR_DTR, 0);

   UrtIntCfg(pADI_UART, COMIEN_ERBFI | COMIEN_ETBEI);
}

void UART_WriteChar(char c)
{
   uart_rdy = 0;
   UrtTx(pADI_UART, c);

   while (uart_rdy == 0);
}

char UART_ReadChar(void)
{
   char c = UrtRx(pADI_UART);

   return c;
}

int _write (int fd, char *ptr, int len)
{
   char *p = ptr;

   int res = UART_SUCCESS;

   (void)fd;
   (void)len;

   while (*p != '\n')
   {
      UART_WriteChar(*p++);
      if (res != UART_SUCCESS)
      {
         break;
      }
   }

   if (*p == '\n')
   {
      UART_WriteChar('\r');
      UART_WriteChar('\n');
   }

   return res;
}

int _read(int fd, char *ptr, int len)
{

   (void)fd;
   (void)len;

   while (uart_read_ch == 0);

   *ptr = uart_rx_char;

   UART_WriteChar(*ptr);
   printf("\n");

   uart_read_ch = 0;

   return 1;
}

void SPI_Init(void)
{
   // CN0397 Chip Select
   DioCfgPin(CN0397_CS_PORT, CN0397_CS_PIN, 0);
   DioPulPin(CN0397_CS_PORT, CN0397_CS_PIN, 0);
   DioOenPin(CN0397_CS_PORT, CN0397_CS_PIN, 1);
   DioSet(CN0397_CS_PORT, CN0397_CS_BIT);

   // CN0398 Chip Select
   DioCfgPin(CN0398_CS_PORT, CN0398_CS_PIN, 0);
   DioPulPin(CN0398_CS_PORT, CN0398_CS_PIN, 0);
   DioOenPin(CN0398_CS_PORT, CN0398_CS_PIN, 1);
   DioSet(CN0398_CS_PORT, CN0398_CS_BIT);

   // CN0370 Chip Select
   //EiCfg(EXTINT3, INT_DIS, INT_RISE);
   //EiCfg(EXTINT4, INT_DIS, INT_RISE);

   DioCfgPin(RED_LED_CS_PORT, RED_LED_CS_PIN, 0);
   DioCfgPin(BLE_LED_CS_PORT, BLE_LED_CS_PIN, 0);
   DioCfgPin(GRN_LED_CS_PORT, GRN_LED_CS_PIN, 0);

   //DioOcePin(BLE_LED_CS_PORT, BLE_LED_CS_PIN, 1);
   //DioOcePin(GRN_LED_CS_PORT, GRN_LED_CS_PIN, 1);

   DioPulPin(RED_LED_CS_PORT, RED_LED_CS_PIN, 1);
   DioPulPin(BLE_LED_CS_PORT, BLE_LED_CS_PIN, 1);
   DioPulPin(GRN_LED_CS_PORT, GRN_LED_CS_PIN, 1);

   DioOenPin(RED_LED_CS_PORT, RED_LED_CS_PIN, 1);
   DioOenPin(BLE_LED_CS_PORT, BLE_LED_CS_PIN, 1);
   DioOenPin(GRN_LED_CS_PORT, GRN_LED_CS_PIN, 1);

   DioSet(RED_LED_CS_PORT, RED_LED_CS_BIT);
   DioSet(BLE_LED_CS_PORT, BLE_LED_CS_BIT);
   DioSet(GRN_LED_CS_PORT, GRN_LED_CS_BIT);

   //SPI 0 MISO
   DioPulPin(pADI_GP1, PIN4, 0);
   DioCfgPin(pADI_GP1, PIN4, 2);
   // SPI 1 MISO
   DioPulPin(pADI_GP0, PIN0, 0);
   DioCfgPin(pADI_GP0, PIN0, 1);

   // SPI 0 SCLK
   DioPulPin(pADI_GP1, PIN5, 0);
   DioCfgPin(pADI_GP1, PIN5, 2);
   // SPI 1 SCLK
   DioPulPin(pADI_GP0, PIN1, 0);
   DioCfgPin(pADI_GP0, PIN1, 1);

   // SPI 0 MOSI
   DioPulPin(pADI_GP1, PIN6, 0);
   DioCfgPin(pADI_GP1, PIN6, 2);
   // SPI 1 MOSI
   DioPulPin(pADI_GP0, PIN2, 0);
   DioCfgPin(pADI_GP0, PIN2, 1);

   // SPI 0 BAUD RATE (115200)
   SpiBaud(pADI_SPI0, 1, SPIDIV_BCRST_DIS);
   // SPI 1 BAUD RATE (115200)
   SpiBaud(pADI_SPI1, 1, SPIDIV_BCRST_DIS);

   /* SPI configuration*/
   SpiCfg(pADI_SPI1, SPICON_MOD_TX4RX4, SPICON_MASEN_EN, SPICON_CON_EN | SPICON_SOEN_EN |
          SPICON_RXOF_EN | SPICON_ZEN_EN | SPICON_TIM_TXWR | SPICON_CPOL_HIGH |
          SPICON_CPHA_SAMPLETRAILING | SPICON_ENABLE_EN);

   SpiCfg(pADI_SPI0, SPICON_MOD_TX2RX2, SPICON_MASEN_EN, SPICON_CON_EN | SPICON_SOEN_EN |
          SPICON_RXOF_EN | SPICON_ZEN_EN | SPICON_TIM_TXWR | SPICON_CPOL_HIGH |
          SPICON_CPHA_SAMPLETRAILING | SPICON_ENABLE_EN);
}

void SPI1_Disable(void)
{
   SpiCfg(pADI_SPI1, SPICON_MOD_TX4RX4, SPICON_MASEN_EN, SPICON_CON_EN | SPICON_SOEN_EN |
          SPICON_RXOF_EN | SPICON_ZEN_EN | SPICON_TIM_TXWR | SPICON_CPOL_HIGH |
          SPICON_CPHA_SAMPLETRAILING | SPICON_ENABLE_DIS);
}

void SPI1_Enable(void)
{
   SpiCfg(pADI_SPI1, SPICON_MOD_TX4RX4, SPICON_MASEN_EN, SPICON_CON_EN | SPICON_SOEN_EN |
          SPICON_RXOF_EN | SPICON_ZEN_EN | SPICON_TIM_TXWR | SPICON_CPOL_HIGH |
          SPICON_CPHA_SAMPLETRAILING | SPICON_ENABLE_EN);
   SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);
   SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);
}

void SPI0_Write(uint16_t data, unsigned char channel)
{
   uint16_t ui16fifo_status = ((2) << 8);

   switch (channel)
   {
      case RED_LED:
         DioClr(RED_LED_CS_PORT, RED_LED_CS_BIT);
         break;
      case BLE_LED:
         DioClr(BLE_LED_CS_PORT, BLE_LED_CS_BIT);
         break;
      case GRN_LED:
         DioClr(GRN_LED_CS_PORT, GRN_LED_CS_BIT);
         break;
   }

   SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

   SpiTx(pADI_SPI0, (uint8_t)((data >> 8) & 0xFF));
   SpiTx(pADI_SPI0, (uint8_t)(data & 0xFF));

   while ((SpiSta(pADI_SPI0) & ui16fifo_status) != ui16fifo_status);
   //spi0TxComplete = 0;

   switch (channel)
   {
      case RED_LED:
         DioSet(RED_LED_CS_PORT, RED_LED_CS_BIT);
         break;
      case BLE_LED:
         DioSet(BLE_LED_CS_PORT, BLE_LED_CS_BIT);
         break;
      case GRN_LED:
         DioSet(GRN_LED_CS_PORT, GRN_LED_CS_BIT);
         break;
   }
}

void SPI_Write(unsigned char* data, unsigned char bytesNumber)
{

    uint8_t byte = 0;

    uint16_t ui16fifo_status = (bytesNumber << 8);                                /* Set FIFO status correct value */

    if(convFlag == 0)
       DioClr(CN0397_CS_PORT, CN0397_CS_BIT);
       timer_sleep(5);

    /* Flush Tx and Rx FIFOs */
    SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

    for (byte = 0; byte < bytesNumber; byte++)
    {
          SpiTx(pADI_SPI1, data[byte]);
    }

    /* Wait until x bytes are received */
    while ((SpiSta(pADI_SPI1) & ui16fifo_status) != ui16fifo_status);

    if(convFlag == 0)
       DioSet(CN0397_CS_PORT, CN0397_CS_BIT);
       timer_sleep(5);

}

void SPI_Read(unsigned char* data, unsigned char bytesNumber)
{

   unsigned char writeData[4]  = {0, 0, 0, 0};
   unsigned char byte          = 0;
   uint16_t ui16fifo_status = ((bytesNumber) << 8);             /* Set FIFO status correct value */

    for (byte = 0; byte <= bytesNumber; byte++)
    {
        if (byte == 0)
           writeData[byte] = data[byte];
        else
           writeData[byte] = 0xAA;    /* dummy value */
    }

    if (convFlag == 0)
    {
       DioClr(CN0397_CS_PORT, CN0397_CS_BIT);
       timer_sleep(5);
    }

    SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

    for (byte = 0; byte <= bytesNumber; byte++)
    {
       SpiTx(pADI_SPI1, writeData[byte]);
    }

    /* Wait until x bytes are received */
    while ((SpiSta(pADI_SPI1) & ui16fifo_status) != ui16fifo_status);

    data[0] = SpiRx(pADI_SPI1);           /* Dummy read, not needed value */

    for (byte = 0; byte < bytesNumber; byte++)
    {
        data[byte] = SpiRx(pADI_SPI1);
    }

    if (convFlag == 0)
    {
       DioSet(CN0397_CS_PORT, CN0397_CS_BIT);
       timer_sleep(5);
    }
}

void SPI1_Write(unsigned char slaveDeviceId, unsigned char *data, unsigned char bytesNumber)
{
   uint8_t byte = 0, reset = 0;

   uint16_t ui16fifo_status;

   if (bytesNumber == 8)
   {
      bytesNumber = 4;
      reset = 1;
   }

   ui16fifo_status = (bytesNumber << 8);                                /* Set FIFO status correct value */

   if (convFlag == 0)
   {
      DioClr(CN0398_CS_PORT, CN0398_CS_BIT);
      timer_sleep(1);
   }

   /* Flush Tx and Rx FIFOs */
   SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

   for(byte = 0;byte < bytesNumber;byte++)
   {
      SpiTx(pADI_SPI1, data[byte]);
   }

   /* Wait until x bytes are received */
   while ((SpiSta(pADI_SPI1) & ui16fifo_status) != ui16fifo_status);

   if (reset == 1)
   {
      SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

      for(byte = 0;byte < bytesNumber;byte++)
      {
            SpiTx(pADI_SPI1, data[byte]);
      }

      while ((SpiSta(pADI_SPI1) & ui16fifo_status) != ui16fifo_status);
   }

   if (convFlag == 0)
   {
     DioSet(CN0398_CS_PORT, CN0398_CS_BIT);
     timer_sleep(1);
   }

}

void SPI1_Read(unsigned char slaveDeviceId, unsigned char* data, unsigned char bytesNumber)
{
   unsigned char writeData[4]  = {0, 0, 0, 0};
   unsigned char byte          = 0;
   uint16_t ui16fifo_status;


   ui16fifo_status = ((bytesNumber) << 8);             /* Set FIFO status correct value */

    for(byte = 0;byte < bytesNumber;byte++)
    {
        if(byte == 0)
           writeData[byte] = data[byte];
        else
           writeData[byte] = 0xAA;    /* dummy value */
    }

    if(convFlag == 0)
    {
       DioClr(CN0398_CS_PORT, CN0398_CS_BIT);
       timer_sleep(1);
    }

    SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

    for(byte = 0;byte < bytesNumber;byte++)
    {
       SpiTx(pADI_SPI1, writeData[byte]);
    }

    /* Wait until x bytes are received */
    while ((SpiSta(pADI_SPI1) & ui16fifo_status) != ui16fifo_status);

    for(byte = 0;byte < bytesNumber;byte++)
    {
        data[byte] = SpiRx(pADI_SPI1);
    }

    if(convFlag == 0)
    {
       DioSet(CN0398_CS_PORT, CN0398_CS_BIT);
       timer_sleep(1);
    }
}

void I2C_Init(void)
{
   // SCL (P2.0)
   DioPulPin(pADI_GP2, PIN0, 0);
   DioCfgPin(pADI_GP2, PIN0, 1);

   // SDA (P2.1)
   DioPulPin(pADI_GP2, PIN1, 0);
   DioCfgPin(pADI_GP2, PIN1, 1);

   // Configure I2C Master (100kHz)
   I2cMCfg(I2CMCON_TXDMA_DIS|I2CMCON_RXDMA_DIS, I2CMCON_IENCMP|I2CMCON_IENRX|I2CMCON_IENTX|I2CMCON_IENALOST_EN|I2CMCON_IENNACK_EN, I2CMCON_MAS_EN);
   I2cBaud(0x4E, 0x4F);
}

uint8_t I2C_Write(uint8_t *ui8Data, uint8_t NumBytes, uint8_t address)
{
   int i;

   // Flush I2C FIFO
   I2cFifoFlush(MASTER, ENABLE);  /* Flush MASTER FIFO */

   // Initialize I2C Transmit Global Variables
   txI2Ccomplete = 0;
   txI2C = 1;

   // Start I2C Transmission of Command
   I2cMWrCfg((address << 1));
   for (i = 0; i < NumBytes; i++)
   {
      I2cTx(MASTER, ui8Data[i]);
   }

   // Wait for Command to Complete
   while (txI2Ccomplete == 0);

   // Re-initialize Command
   txI2Ccomplete = 0;
   txI2C = 0;

   return 0;
}

uint8_t I2C_Read(uint8_t NumBytes, uint8_t address)
{
   unsigned int uiStatus, i;

   // Flush FIFO
   I2cFifoFlush(MASTER, ENABLE);

   // Check If I2C Line is Busy
   uiStatus = I2cSta(MASTER);
   while ((uiStatus & I2CMSTA_BUSY_SET) == I2CMSTA_BUSY_SET)
   {
         uiStatus = I2cSta(MASTER);
   }
   timer_sleep(5);

   // Initialize I2C Read Global Variables
   rxI2Ccomplete = 0;
   rxI2C = 0;
   rxI2Csize = NumBytes;
   rxI2Cbuf = (uint8_t *)malloc(NumBytes * sizeof(uint8_t));

   for (i = 0; i < NumBytes; i++)
   {
         rxI2Cbuf[i] = 0;
   }

   // Initiate I2C Read Command
   I2cMRdCfg((address << 1), NumBytes, DISABLE);

   // Wait for I2C Read Completion
   while (rxI2Ccomplete != 2);

   // Print Raw Data
   if (I2C_PRINT_RAW)
   {
      for (i = 0; i < rxI2Csize; i++)
      {
           printf("Buffer[%d]: 0x%x\n", i, rxI2Cbuf[i]);
      }
   }

   // Re-initialize I2C Global Variables
   rxI2Ccomplete = 0;
   rxI2C = 0;
   rxI2Csize = 0;

   return 0;

}
