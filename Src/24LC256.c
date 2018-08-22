/*
 * 24LC256.c
 *
 *  Created on: Jul 29, 2018
 *      Author: Sergey
 */
#include "24LC256.h"

uint8_t timeout;

void WriteProtection(bool state){
	if(state){
		LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_8);
	}else{
		LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_8);
	}
	return;
}

I2C_RequestResult Waiting4Flag (Flags flag, I2C_Timeout SetTimeout){
	I2C_RequestResult result;
	timeout = SetTimeout;
	LL_SYSTICK_EnableIT();
	if(flag == Busy){
		while (LL_I2C_IsActiveFlag_BUSY(I2C1) && timeout){}
		if (LL_I2C_IsActiveFlag_BUSY(I2C1)){
			result = I2C_Busy_now;
		}else {
			result = I2C_Success;
		}
	}
	if(flag==Address){
		while (!LL_I2C_IsActiveFlag_ADDR(I2C1) && timeout){}
		if(!LL_I2C_IsActiveFlag_ADDR(I2C1)){
			result = I2C_Address_error;
		}else{
			result = I2C_Success;
		}
	}
	if(flag==StartBit){
		while (!LL_I2C_IsActiveFlag_SB(I2C1) && timeout){}
		if(!LL_I2C_IsActiveFlag_SB(I2C1)){
			result = I2C_StartBit_error;
		}else{
			result = I2C_Success;
		}
	}
	if(flag==ByteTransfer){
		while (!LL_I2C_IsActiveFlag_BTF(I2C1) && timeout){}
		if(!LL_I2C_IsActiveFlag_BTF(I2C1)){
			result=I2C_ByteTransfer_error;
		}else{
			result=I2C_Success;
		}
	}
	if(flag==RXNE){
		while (!LL_I2C_IsActiveFlag_RXNE(I2C1) && timeout){}
		if(!LL_I2C_IsActiveFlag_RXNE(I2C1)){
			result=I2C_RXNE_error;
		}else{
			result=I2C_Success;
		}
	}
	LL_SYSTICK_DisableIT();
	return result;
}

I2C_RequestResult EEPROM_I2C_mem_WritePage64 (I2CReq *R){
	I2C_RequestResult result;
	WriteProtection(false);
	if ((result = Waiting4Flag(Busy,R->timeout)) != I2C_Success){
		return result;
	}

	LL_I2C_GenerateStartCondition(I2C1);

	if( (result = Waiting4Flag(StartBit,R->timeout)) != I2C_Success){
		return result;
	}
	LL_I2C_TransmitData8 (I2C1, R->SlaveAddr|I2C_WRITE);

	if((result = Waiting4Flag(Address,R->timeout)) != I2C_Success){
		return result;
	}
	LL_I2C_ClearFlag_ADDR(I2C1);

	LL_I2C_TransmitData8 (I2C1, R->InternalAddr.addr8[1]);

	if( (result = Waiting4Flag(ByteTransfer,R->timeout)) != I2C_Success){
		return result;
	}

	LL_I2C_TransmitData8 (I2C1, R->InternalAddr.addr8[0]);
	while (!LL_I2C_IsActiveFlag_BTF(I2C1)){}
	if ((result = Waiting4Flag(ByteTransfer,R->timeout)) != I2C_Success){
		return result;
	}

	for (uint16_t i = 0; i<(PageSize); i++){
		LL_I2C_TransmitData8 (I2C1, R->data[R->dataCounter++]);
		if ((result = Waiting4Flag(ByteTransfer,R->timeout)) != I2C_Success){
			return result;
		}
		R->InternalAddr.addr16++;
		if(R->InternalAddr.addr16 == ByteQuantity-1){
			R->InternalAddr.addr16 = 0;
		}
	}
	LL_I2C_GenerateStopCondition(I2C1);
	WriteProtection(true);
	return result;
}

I2C_RequestResult EEPROM_I2C_mem_Write (I2CReq * R){
	I2C_RequestResult result;
	uint8_t RequestsQuantity;

	if (R->DataAmount < PageSize){
		RequestsQuantity = 1;
	}else{
		RequestsQuantity = R->DataAmount/PageSize;
	}

	for(uint8_t j = 0; j<RequestsQuantity; j++){
		if((result = EEPROM_I2C_mem_WritePage64 (R)) == !I2C_Success){
			break;
		}
		LL_mDelay(5);
	}

	return result;

}

I2C_RequestResult EEPROM_I2C_mem_Read (I2CReq *R, I2C_READING_TYPE type){

		I2C_RequestResult result;
		if ((type == CURRENT || type == RANDOM) && R->DataAmount>1) {
			return I2C_Init_error;
		}

		if ((result = Waiting4Flag(Busy,R->timeout)) != I2C_Success){
			return result;
		}

		LL_I2C_GenerateStartCondition(I2C1);

		if((result = Waiting4Flag(StartBit,R->timeout)) != I2C_Success){
			return result;
		}

		if (type == CURRENT){
			LL_I2C_TransmitData8 (I2C1, R->SlaveAddr|I2C_READ);
		}
		else {
			LL_I2C_TransmitData8 (I2C1, R->SlaveAddr|I2C_WRITE);
		}

		if((result = Waiting4Flag(Address,R->timeout)) != I2C_Success){
			return result;
		}
		LL_I2C_ClearFlag_ADDR(I2C1);


		if (type == RANDOM || type == SEQUENTIAL){
			LL_I2C_TransmitData8 (I2C1, R->InternalAddr.addr8[1]);

			if ((result = Waiting4Flag(ByteTransfer,R->timeout)) != I2C_Success){
				return result;
			}
			LL_I2C_TransmitData8 (I2C1, R->InternalAddr.addr8[0]);

			if ((result = Waiting4Flag(ByteTransfer,R->timeout)) != I2C_Success){
				return result;
			}
			LL_I2C_GenerateStartCondition(I2C1);

			if((result = Waiting4Flag(StartBit,R->timeout)) != I2C_Success){
				return result;
			}
			LL_I2C_TransmitData8 (I2C1, R->SlaveAddr|I2C_READ);

			if((result = Waiting4Flag(Address,R->timeout)) != I2C_Success){
				return result;
			}
			LL_I2C_ClearFlag_ADDR(I2C1);
		}

		for (uint8_t i = 0; i<R->DataAmount; i++){
			if (i==(R->DataAmount-1)){
				LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
			}
			else {
				LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
			}

			if((result = Waiting4Flag(RXNE,R->timeout)) != I2C_Success){
				return result;
			}
			R->data[i] = LL_I2C_ReceiveData8(I2C1);
		}
		LL_I2C_GenerateStopCondition(I2C1);
	return result;
}
