/*
 * softwareCan.h
 *
 *  Created on: Aug 9, 2025
 *      Author: cezar
 */

#ifndef INC_SOFTWARECAN_H_
#define INC_SOFTWARECAN_H_

typedef struct{
	TIM_HandleTypeDef *tim;
	GPIO_TypeDef 	  *gpioPort;
	uint16_t		  pin;
} initStruct_t;

extern initStruct_t canConfig;

typedef enum{
	ERROR_PARAM = -1,
	PARAM_OK
} sendCanFrameStatus_t;

void delayUs(uint32_t us);
sendCanFrameStatus_t initSoftwareCan(TIM_HandleTypeDef *tim, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
sendCanFrameStatus_t sendCanFrame(uint16_t id, uint8_t dlc, uint8_t *data);

#endif /* INC_SOFTWARECAN_H_ */
