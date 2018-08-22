/*
 * 24LC256.h
 *
 *  Created on: Jul 29, 2018
 *      Author: Sergey
 */
//#include "stm32f4xx_ll_i2c.h"
//#include "stm32f4xx_ll_cortex.h"
//#include "stm32f4xx_ll_utils.h"
#include "main.h"
#include "stdbool.h"

typedef enum {
	I2C_WRITE = 0x00,
	I2C_READ  = 0x01,
	SLAVE_ADDR = 0xA0,
}I2C;

typedef enum{
	PageSize = 64,
	PagesQuantity = 500,
	ByteQuantity = 32000
}MemoryFeatures;

//typedef struct {
//
//}Pages;

typedef	enum {
	CURRENT,
	RANDOM,
	SEQUENTIAL
}I2C_READING_TYPE;

typedef enum{
	TIMEOUT_10ms = 1,
	TIMEOUT_50ms = 5,
	TIMEOUT_100ms = 10
}I2C_Timeout;

 typedef struct{
	uint8_t SlaveAddr;
	union{
		uint16_t addr16;
		uint8_t addr8 [2];
	}InternalAddr;
	uint8_t DataAmount;
	uint8_t * data;
	uint8_t dataCounter;
	I2C_Timeout timeout;
}I2CReq;

typedef enum {
	StartBit,
	Address,
	ByteTransfer,
	RXNE,
	Busy
}Flags;

typedef enum {
	I2C_StartBit_error,
	I2C_Address_error,
	I2C_ByteTransfer_error,
	I2C_RXNE_error,
	I2C_Busy_now,
	I2C_Success,
	I2C_Init_error
}I2C_RequestResult;

void WriteProtection(bool state);
I2C_RequestResult Waiting4Flag (Flags flag, I2C_Timeout SetTimeout);
I2C_RequestResult EEPROM_I2C_mem_WritePage64 (I2CReq * R);
I2C_RequestResult EEPROM_I2C_mem_Write (I2CReq * R);
I2C_RequestResult EEPROM_I2C_mem_Read (I2CReq *R, I2C_READING_TYPE type);
