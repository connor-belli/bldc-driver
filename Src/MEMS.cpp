#include "MEMS.h"
#include <stm32f303xc.h>

static GYRO_DrvTypeDef* GyroscopeDrv;

void GYRO_IO_Init(void) {

}

void GYRO_IO_DeInit(void) {

}

void GYRO_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr,
		uint16_t NumByteToWrite) {
	if (NumByteToWrite > 1) {
		WriteAddr |= 0x40;
	}
	GPIOE->BSRR = 1 << (3 + 16);

	while (!(SPI1->SR & SPI_SR_TXE))
		;
	(*((__IO uint8_t*) &SPI1->DR)) = WriteAddr;

	while (!(SPI1->SR & SPI_SR_RXNE))
		;
	uint32_t dummy __attribute__((unused)) = *(__IO uint8_t*) SPI1->DR;

	for (int i = 0; i < NumByteToWrite; i++) {
		while (!(SPI1->SR & SPI_SR_TXE))
			;
		(*((__IO uint8_t*) &SPI1->DR)) = pBuffer[i];

		while (!(SPI1->SR & SPI_SR_RXNE))
			;
		int dummy __attribute__((unused)) = *((__IO uint8_t*) &SPI1->DR);
	}
	GPIOE->BSRR = 1 << 3;
}

void GYRO_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead) {
	ReadAddr |= 0x80;
	if (NumByteToRead > 1) {
		ReadAddr |= 0x40;
	}
	GPIOE->BSRR = 1 << (3 + 16);

	while (!(SPI1->SR & SPI_SR_TXE))
		;
	(*((__IO uint8_t*) &SPI1->DR)) = ReadAddr;

	while (!(SPI1->SR & SPI_SR_RXNE))
		;
	uint32_t dummy __attribute__((unused)) = *(__IO uint8_t*) SPI1->DR;

	for (int i = 0; i < NumByteToRead; i++) {
		while (!(SPI1->SR & SPI_SR_TXE))
			;
		(*((__IO uint8_t*) &SPI1->DR)) = 0x0;

		while (!(SPI1->SR & SPI_SR_RXNE))
			;
		uint32_t x = *((__IO uint8_t*) &SPI1->DR);
		pBuffer[i] = x;
	}
	GPIOE->BSRR = 1 << 3;
}

// Waits for flag to be reset in the ISR register
// Returns true if there is an error on I2C1
bool I2C_check_error() {
	// If an Arbitration lost error occurs, restart I2C bus
	if(I2C1->ISR & I2C_ISR_ARLO) {
		I2C1->CR1 &= ~I2C_CR1_PE;
		while((I2C1->ISR & I2C_ISR_BUSY))
			;
		I2C1->CR1 |= I2C_CR1_PE;
		return true;
	} else if(I2C1->ISR & I2C_ISR_NACKF) {
		// Else if it is just a NACK, re send message
		return true;
	} else {
		// No error occurred
		return false;
	}
}

bool wait_I2C_set(uint32_t flag) {
	while(!(I2C1->ISR & flag)) {
		if(I2C_check_error()) return true;
	}
	return false;
}

// Waits for flag to be set in the ISR register
// Returns true if the message needs to be resent due to fault
bool wait_I2C_reset(uint32_t flag) {
	while(I2C1->ISR & flag) {
		if(I2C_check_error()) return true;
	}
	return false;
}


void    COMPASSACCELERO_IO_Init(void) {

}
void    COMPASSACCELERO_IO_ITConfig(void) {
	// Set external interrupts for 4, 5
	//SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PE |SYSCFG_EXTICR1_EXTI1_PE;
	//SYSCFG->EXTICR[0] |= SYSCFG_EXTICR2_EXTI4_PE | SYSCFG_EXTICR2_EXTI5_PE;
	// Enable rising edge events
	//EXTI->RTSR |= 0x30;
	// Enable external events
	//EXTI->EMR |= 0x30;
}
void    COMPASSACCELERO_IO_Write(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t Value) {
	// Loop infinitely until message is successfully sent
	for(;;) {
		// Wait for I2C bus to be clear
		if(wait_I2C_reset(I2C_ISR_BUSY)) continue;
		// Send Start of I2C message
		I2C1->CR2 = ((uint32_t)DeviceAddr & I2C_CR2_SADD) | I2C_CR2_RELOAD |
				I2C_CR2_START | (1 << I2C_CR2_NBYTES_Pos);
		// Wait for ACK
		if(wait_I2C_set(I2C_ISR_TXIS)) continue;
		// Write register address
		I2C1->TXDR = RegisterAddr;
		// Wait for transfer to complete
		if(wait_I2C_set(I2C_ISR_TCR)) continue;
		// Signal the message to end and write one byte
		I2C1->CR2 = ((uint32_t)DeviceAddr & I2C_CR2_SADD) | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND;
		// Write data to be written to the bus
		I2C1->TXDR = Value;
		// Wait for I2C to detect stop bit
		if(wait_I2C_set(I2C_ISR_STOPF)) continue;
		// Clear stop detection bit
		I2C1->ICR = I2C_ICR_STOPCF;
		return;
	}
}



uint8_t COMPASSACCELERO_IO_Read(uint16_t DeviceAddr, uint8_t RegisterAddr) {
	for(;;) {
		// Wait for I2C to be not busy
		if(wait_I2C_reset(I2C_ISR_BUSY)) continue;
		// Send Start of I2C message
		I2C1->CR2 = ((uint32_t)DeviceAddr & I2C_CR2_SADD) |
				I2C_CR2_START | (1 << I2C_CR2_NBYTES_Pos);
		// Wait to be ACK
		if(wait_I2C_set(I2C_ISR_TXIS)) continue;
		// Write register address
		I2C1->TXDR = RegisterAddr;
		// Wait for transfer to complete
		if(wait_I2C_set(I2C_ISR_TC)) continue;
		// Signal the message to end and read 1 byte
		I2C1->CR2 = ((uint32_t)DeviceAddr & I2C_CR2_SADD) | I2C_CR2_RD_WRN
				| I2C_CR2_START | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND;
		// Wait for data to be received
		if(wait_I2C_set(I2C_ISR_RXNE)) continue;
		// Read received data
		uint8_t result = (uint8_t)I2C1->RXDR;
		// Wait for stop to be detected
		if(wait_I2C_set(I2C_ISR_STOPF)) continue;
		// Clear stop detection bit
		I2C1->ICR = I2C_ICR_STOPCF;
		return result;
	}
}

void init_gyro() {
	  uint16_t ctrl = 0x0000;
	  GYRO_InitTypeDef         Gyro_InitStructure;
	  GYRO_FilterConfigTypeDef Gyro_FilterStructure = {0,0};
	 GyroscopeDrv = &I3g4250Drv;

	/* Configure Mems : data rate, power mode, full scale and axes */
	Gyro_InitStructure.Power_Mode       = I3G4250D_MODE_ACTIVE;
	Gyro_InitStructure.Output_DataRate  = I3G4250D_OUTPUT_DATARATE_1;
	Gyro_InitStructure.Axes_Enable      = I3G4250D_AXES_ENABLE;
	Gyro_InitStructure.Band_Width       = I3G4250D_BANDWIDTH_4;
	Gyro_InitStructure.BlockData_Update = I3G4250D_BlockDataUpdate_Continous;
	Gyro_InitStructure.Endianness       = I3G4250D_BLE_LSB;
	Gyro_InitStructure.Full_Scale       = I3G4250D_FULLSCALE_500;

	/* Configure MEMS: data rate, power mode, full scale and axes */
	ctrl = (uint16_t) (Gyro_InitStructure.Power_Mode  | Gyro_InitStructure.Output_DataRate | \
					   Gyro_InitStructure.Axes_Enable | Gyro_InitStructure.Band_Width);

	ctrl |= (uint16_t) ((Gyro_InitStructure.BlockData_Update | Gyro_InitStructure.Endianness | \
						 Gyro_InitStructure.Full_Scale) << 8);

	/* Initialize the gyroscope */
	GyroscopeDrv->Init(ctrl);

	Gyro_FilterStructure.HighPassFilter_Mode_Selection   = I3G4250D_HPM_NORMAL_MODE_RES;
	Gyro_FilterStructure.HighPassFilter_CutOff_Frequency = I3G4250D_HPFCF_0;

	ctrl = (uint8_t) ((Gyro_FilterStructure.HighPassFilter_Mode_Selection |\
					   Gyro_FilterStructure.HighPassFilter_CutOff_Frequency));

	/* Configure the gyroscope main parameters */
	GyroscopeDrv->FilterConfig(ctrl);

	GyroscopeDrv->FilterCmd(I3G4250D_HIGHPASSFILTER_ENABLE);
}

void init_accel() {
	  uint16_t ctrl = 0x0000;
	  ACCELERO_InitTypeDef         Accelero_InitStructure;
	  ACCELERO_FilterConfigTypeDef Accelero_FilterStructure = {};
	/* Initialize the accelerometer driver structure */
	ACCELERO_DrvTypeDef* AccelerometerDrv = &Lsm303dlhcDrv;

	/* MEMS configuration ----------------------------------------------------*/
	/* Fill the accelerometer structure */
	Accelero_InitStructure.Power_Mode         = LSM303DLHC_NORMAL_MODE;
	Accelero_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_1344_HZ;
	Accelero_InitStructure.Axes_Enable        = LSM303DLHC_AXES_ENABLE;
	Accelero_InitStructure.AccFull_Scale      = LSM303DLHC_FULLSCALE_2G;
	Accelero_InitStructure.BlockData_Update   = LSM303DLHC_BlockUpdate_Continous;
	Accelero_InitStructure.Endianness         = LSM303DLHC_BLE_LSB;
	Accelero_InitStructure.High_Resolution    = LSM303DLHC_HR_ENABLE;

	/* Configure MEMS: data rate, power mode, full scale and axes */
	ctrl |= (Accelero_InitStructure.Power_Mode | Accelero_InitStructure.AccOutput_DataRate | \
			 Accelero_InitStructure.Axes_Enable);

	ctrl |= ((Accelero_InitStructure.BlockData_Update | Accelero_InitStructure.Endianness | \
			  Accelero_InitStructure.AccFull_Scale    | Accelero_InitStructure.High_Resolution) << 8);

	/* Configure the accelerometer main parameters */
	AccelerometerDrv->Init(ctrl);

	/* Fill the accelerometer LPF structure */
	Accelero_FilterStructure.HighPassFilter_Mode_Selection   = LSM303DLHC_HPM_NORMAL_MODE;
	Accelero_FilterStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
	Accelero_FilterStructure.HighPassFilter_AOI1             = LSM303DLHC_HPF_AOI1_DISABLE;
	Accelero_FilterStructure.HighPassFilter_AOI2             = LSM303DLHC_HPF_AOI2_DISABLE;

	/* Configure MEMS: mode, cutoff frquency, Filter status, Click, AOI1 and AOI2 */
	ctrl = (uint8_t) (Accelero_FilterStructure.HighPassFilter_Mode_Selection   |\
					  Accelero_FilterStructure.HighPassFilter_CutOff_Frequency |\
					  Accelero_FilterStructure.HighPassFilter_AOI1             |\
					  Accelero_FilterStructure.HighPassFilter_AOI2);

	/* Configure the accelerometer LPF main parameters */
	AccelerometerDrv->FilterConfig(ctrl);
	LSM303DLHC_AccINT1InterruptDisable(0xFF, 0xFF);
	LSM303DLHC_AccINT2InterruptDisable(0xFF, 0xFF);
}

void MEMS_init() {
	// Set GPIO 0-5 up for MEMS sensors
	// Set 0-5 to be high speed
	GPIOE->OSPEEDR |= 0xFFF;
	GPIOE->BSRR = 1 << 3;


	// Configure SPI
	// Set A5-A7 to alternate function
	GPIOA->MODER |= 0x2A << 10;
	// Set A5-A7 to high speed
	GPIOA->OSPEEDR |= 0x3F << 10;
	// Set Alternate function to SPI1
	GPIOA->AFR[0] |= 0x555 << 20;
	// Set pre-scaler to divide by 16: 72 MHz / 16 = 6 MHz BR
	SPI1->CR1 |= 3 << SPI_CR1_BR_Pos;
	// Set device to be master
	SPI1->CR1 |= SPI_CR1_MSTR;
	// Set data size to 8 bits
	SPI1->CR2 |= 7 << SPI_CR2_DS_Pos;
	// Set RXFIFO threshold to be 8bits;
	SPI1->CR2 |= SPI_CR2_FRXTH;
	// Set devices to do slave management
	SPI1->CR1 |= SPI_CR1_SSM;
	SPI1->CR1 |= SPI_CR1_SSI;
	// Enable SPI
	SPI1->CR1 |= SPI_CR1_SPE;


	//Configure I2C
	// Set B6, 7 to alternate mode
	GPIOB->MODER |= 0xA << 12;
	// Set 6, 7 to high speed
	GPIOB->OSPEEDR |= 0xF << 12;
	// Set alternate function to I2C1
	GPIOB->AFR[0] |= 0x44 << 24;

	// set Own address 1 to 0x32 and enable it
	I2C1->OAR1 |= 0x32 | I2C_OAR1_OA1EN;
	//Enable Autoend and NACK
	I2C1->CR2 |= I2C_CR2_AUTOEND | I2C_CR2_NACK;
	// Enable I2C1
	I2C1->CR1 |= I2C_CR1_PE;


	// Initialize Gyro and Accel
	init_gyro();
	init_accel();
}

