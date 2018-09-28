/***************************************************************************//**
*   @file    AD7124.c
*   @brief   AD7124 implementation file.
*   @devices AD7124-4, AD7124-8
*
********************************************************************************
* Copyright 2015(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <AD7124.h>
#include <Communication.h>

/***************************************************************************//**
* @brief Reads the value of the specified register without checking if the
*        device is ready to accept user requests.
*
* @param device - The handler of the instance of the driver.
* @param pReg - Pointer to the register structure holding info about the
*               register to be read. The read value is stored inside the
*               register structure.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_NoCheckReadRegister(struct ad7124_device *device,
				   struct ad7124_st_reg* pReg)
{
	int32_t ret       = 0;
	uint8_t buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t i         = 0;
	uint8_t check8    = 0;
	uint8_t msgBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	if(!device || !pReg)
		return INVALID_VAL;

	/* Build the Command word */
	buffer[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD |
		    AD7124_COMM_REG_RA(pReg->addr);

	/* Read data from the device */
	ret = SPI_Read(device->slave_select_id,
		       buffer,
		       ((device->useCRC != AD7124_DISABLE_CRC) ? pReg->size + 1
			: pReg->size) + 1);
	if(ret < 0)
		return ret;

	/* Check the CRC */
	if(device->useCRC == AD7124_USE_CRC) {
		msgBuf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD |
			    AD7124_COMM_REG_RA(pReg->addr);
		for(i = 1; i < pReg->size + 2; ++i) {
			msgBuf[i] = buffer[i];
		}
		check8 = AD7124_ComputeCRC8(msgBuf, pReg->size + 2);
	}

	if(check8 != 0) {
		/* ReadRegister checksum failed. */
		return COMM_ERR;
	}

	/* Build the result */
	pReg->value = 0;
	for(i = 1; i < pReg->size + 1; i++) {
		pReg->value <<= 8;
		pReg->value += buffer[i];
	}

	return ret;
}

/***************************************************************************//**
* @brief Writes the value of the specified register without checking if the
*        device is ready to accept user requests.
*
* @param device - The handler of the instance of the driver.
* @param reg - Register structure holding info about the register to be written
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_NoCheckWriteRegister(struct ad7124_device *device,
				    struct ad7124_st_reg reg)
{
	int32_t ret      = 0;
	int32_t regValue = 0;
	uint8_t wrBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t i        = 0;
	uint8_t crc8     = 0;

	if(!device)
		return INVALID_VAL;

	/* Build the Command word */
	wrBuf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_WR |
		   AD7124_COMM_REG_RA(reg.addr);

	/* Fill the write buffer */
	regValue = reg.value;
	for(i = 0; i < reg.size; i++) {
		wrBuf[reg.size - i] = regValue & 0xFF;
		regValue >>= 8;
	}

	/* Compute the CRC */
	if(device->useCRC != AD7124_DISABLE_CRC) {
		crc8 = AD7124_ComputeCRC8(wrBuf, reg.size + 1);
		wrBuf[reg.size + 1] = crc8;
	}

	/* Write data to the device */
	ret = SPI_Write(device->slave_select_id,
			wrBuf,
			(device->useCRC != AD7124_DISABLE_CRC) ? reg.size + 2
			: reg.size + 1);

	return ret;
}

/***************************************************************************//**
* @brief Reads the value of the specified register only when the device is ready
*        to accept user requests. If the device ready flag is deactivated the
*        read operation will be executed without checking the device state.
*
* @param device - The handler of the instance of the driver.
* @param pReg - Pointer to the register structure holding info about the
*               register to be read. The read value is stored inside the
*               register structure.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_ReadRegister(struct ad7124_device *device,
			    struct ad7124_st_reg* pReg)
{
	int32_t ret;

	if (pReg->addr != AD7124_ERR_REG && device->check_ready) {
		ret = AD7124_WaitForSpiReady(device, device->spi_rdy_poll_cnt);
		if (ret < 0)
			return ret;
	}
	ret = AD7124_NoCheckReadRegister(device, pReg);

	return ret;
}

/***************************************************************************//**
* @brief Writes the value of the specified register only when the device is
*        ready to accept user requests. If the device ready flag is deactivated
*        the write operation will be executed without checking the device state.
*
* @param device - The handler of the instance of the driver.
* @param reg - Register structure holding info about the register to be written
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_WriteRegister(struct ad7124_device *device,
			     struct ad7124_st_reg pReg)
{
	int32_t ret;

	if (device->check_ready) {
		ret = AD7124_WaitForSpiReady(device, device->spi_rdy_poll_cnt);
		if (ret < 0)
			return ret;
	}
	ret = AD7124_NoCheckWriteRegister(device, pReg);

	return ret;
}

/***************************************************************************//**
* @brief Resets the device.
*
* @param device - The handler of the instance of the driver.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_Reset(struct ad7124_device *device)
{
	int32_t ret = 0;
	uint8_t wrBuf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	if(!device)
		return INVALID_VAL;

	ret = SPI_Write(device->slave_select_id, wrBuf, 8);

	/* Wait for the reset to complete */
	ret = AD7124_WaitToPowerOn(device, device->spi_rdy_poll_cnt);

	return ret;
}

/***************************************************************************//**
* @brief Waits until the device can accept read and write user actions.
*
* @param device - The handler of the instance of the driver.
* @param timeout - Count representing the number of polls to be done until the
*                  function returns.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_WaitForSpiReady(struct ad7124_device *device, uint32_t timeout)
{
	struct ad7124_st_reg *regs;
	int32_t ret;
	int8_t ready = 0;

	if(!device)
		return INVALID_VAL;

	regs = device->regs;

	while(!ready && --timeout) {
		/* Read the value of the Error Register */
		ret = AD7124_ReadRegister(device, &regs[AD7124_Error]);
		if(ret < 0)
			return ret;

		/* Check the SPI IGNORE Error bit in the Error Register */
		ready = (regs[AD7124_Error].value &
			 AD7124_ERR_REG_SPI_IGNORE_ERR) == 0;
	}

	return timeout ? 0 : TIMEOUT;
}

/***************************************************************************//**
* @brief Waits until the device finishes the power-on reset operation.
*
* @param device - The handler of the instance of the driver.
* @param timeout - Count representing the number of polls to be done until the
*                  function returns.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_WaitToPowerOn(struct ad7124_device *device, uint32_t timeout)
{
	struct ad7124_st_reg *regs;
	int32_t ret;
	int8_t powered_on = 0;

	if(!device)
		return INVALID_VAL;

	regs = device->regs;

	while(!powered_on && timeout--) {
		ret = AD7124_ReadRegister(device, &regs[AD7124_Status]);
		if(ret < 0)
			return ret;

		/* Check the POR_FLAG bit in the Status Register */
		powered_on = (regs[AD7124_Status].value &
			      AD7124_STATUS_REG_POR_FLAG) == 0;
	}

	return (timeout || powered_on) ? 0 : TIMEOUT;
}

/***************************************************************************//**
* @brief Waits until a new conversion result is available.
*
* @param device - The handler of the instance of the driver.
* @param timeout - Count representing the number of polls to be done until the
*                  function returns if no new data is available.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_WaitForConvReady(struct ad7124_device *device, uint32_t timeout)
{
	struct ad7124_st_reg *regs;
	int32_t ret;
	int8_t ready = 0;

	if(!device)
		return INVALID_VAL;

	regs = device->regs;

	while(!ready && --timeout) {
		/* Read the value of the Status Register */
		ret = AD7124_ReadRegister(device, &regs[AD7124_Status]);
		if(ret < 0)
			return ret;

		/* Check the RDY bit in the Status Register */
		ready = (regs[AD7124_Status].value &
			 AD7124_STATUS_REG_RDY) == 0;
	}

	return timeout ? 0 : TIMEOUT;
}

/***************************************************************************//**
* @brief Reads the conversion result from the device.
*
* @param device - The handler of the instance of the driver.
* @param pData - Pointer to store the read data.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_ReadData(struct ad7124_device *device, int32_t* pData)
{
	struct ad7124_st_reg *regs;
	int32_t ret;

	if(!device)
		return INVALID_VAL;

	regs = device->regs;

	/* Read the value of the Data Register */
	ret = AD7124_ReadRegister(device, &regs[AD7124_Data]);

	/* Get the read result */
	*pData = regs[AD7124_Data].value;

	return ret;
}

/***************************************************************************//**
* @brief Computes the CRC checksum for a data buffer.
*
* @param pBuf - Data buffer
* @param bufSize - Data buffer size in bytes
*
* @return Returns the computed CRC checksum.
*******************************************************************************/
uint8_t AD7124_ComputeCRC8(uint8_t * pBuf, uint8_t bufSize)
{
	uint8_t i   = 0;
	uint8_t crc = 0;

	while(bufSize) {
		for(i = 0x80; i != 0; i >>= 1) {
			if(((crc & 0x80) != 0) != ((*pBuf & i) !=
						   0)) { /* MSB of CRC register XOR input Bit from Data */
				crc <<= 1;
				crc ^= AD7124_CRC8_POLYNOMIAL_REPRESENTATION;
			} else {
				crc <<= 1;
			}
		}
		pBuf++;
		bufSize--;
	}
	return crc;
}

/***************************************************************************//**
* @brief Updates the CRC settings.
*
* @param device - The handler of the instance of the driver.
*
* @return None.
*******************************************************************************/
void AD7124_UpdateCRCSetting(struct ad7124_device *device)
{
	struct ad7124_st_reg *regs;

	if(!device)
		return;

	regs = device->regs;

	/* Get CRC State. */
	if (regs[AD7124_Error_En].value & AD7124_ERREN_REG_SPI_CRC_ERR_EN) {
		device->useCRC = AD7124_USE_CRC;
	} else {
		device->useCRC = AD7124_DISABLE_CRC;
	}
}

/***************************************************************************//**
* @brief Updates the device SPI interface settings.
*
* @param device - The handler of the instance of the driver.
*
* @return None.
*******************************************************************************/
void AD7124_UpdateDevSpiSettings(struct ad7124_device *device)
{
	struct ad7124_st_reg *regs;

	if(!device)
		return;

	regs = device->regs;

	if (regs[AD7124_Error_En].value & AD7124_ERREN_REG_SPI_IGNORE_ERR_EN) {
		device->check_ready = 1;
	} else {
		device->check_ready = 0;
	}
}

/***************************************************************************//**
* @brief Initializes the AD7124.
*
* @param device - The handler of the instance of the driver.
* @param opMode - Operation Mode of the ADC
*******************************************************************************/
void AD7124_OperationMode(struct ad7124_device *device, enum op_mode mode)
{
	AD7124_ReadRegister(device, &device->regs[AD7124_ADC_Control]);
	device->regs[AD7124_ADC_Control].value = ((
				device->regs[AD7124_ADC_Control].value & 0xFFC3) | AD7124_ADC_CTRL_REG_MODE(
				mode));
	AD7124_WriteRegister(device, device->regs[AD7124_ADC_Control]);
}


/***************************************************************************//**
* @brief Initializes the AD7124.
*
* @param device - The handler of the instance of the driver.
* @param slave_select - The Slave Chip Select Id to be passed to the SPI calls.
* @param regs - The list of registers of the device (initialized or not) to be
*               added to the instance of the driver.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_Setup(struct ad7124_device *device, int slave_select_id,
		     struct ad7124_st_reg *regs)
{
	int32_t ret;
	enum ad7124_registers regNr;

	if(!device || !regs)
		return INVALID_VAL;

	device->regs = regs;
	device->slave_select_id = slave_select_id;
	device->spi_rdy_poll_cnt = AD7124_SPI_RDY_POLL_CNT;

	/*  Reset the device interface.*/
	ret = AD7124_Reset(device);
	if (ret < 0)
		return ret;

	/* Update the device structure with power-on/reset settings */
	device->check_ready = 1;

	/* Initialize registers AD7124_ADC_Control through AD7124_Filter_7. */
	for(regNr = AD7124_Status; (regNr < AD7124_Offset_0) && !(ret < 0);
	    regNr++) {
		if (regs[regNr].rw == AD7124_RW) {
			ret = AD7124_WriteRegister(device, regs[regNr]);
			if (ret < 0)
				break;
		}

		/* Get CRC State and device SPI interface settings */
		if (regNr == AD7124_Error_En) {
			AD7124_UpdateCRCSetting(device);
			AD7124_UpdateDevSpiSettings(device);
		}
	}

	return ret;
}
