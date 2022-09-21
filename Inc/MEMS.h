#pragma once
#include <stdint.h>
#include "Components/i3g4250d/i3g4250d.h"
#include "Components/lsm303dlhc/lsm303dlhc.h"

// Uses pins A5-A7, B6-B7, E0-E5

extern "C" {
	void    GYRO_IO_Init(void);
	void    GYRO_IO_DeInit(void);
	void    GYRO_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
	void    GYRO_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
	void    COMPASSACCELERO_IO_Init(void);
	void    COMPASSACCELERO_IO_ITConfig(void);
	void    COMPASSACCELERO_IO_Write(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t Value);
	uint8_t COMPASSACCELERO_IO_Read(uint16_t DeviceAddr, uint8_t RegisterAddr);
}

void MEMS_init();
