/*
 * softwareCan.h
 *
 *  Created on: Aug 9, 2025
 *      Author: cezar
 */

#ifndef INC_SOFTWARECAN_H_
#define INC_SOFTWARECAN_H_

#include <stdbool.h>

#define MAX_BITS 256

typedef struct{
	TIM_HandleTypeDef *tim;
	GPIO_TypeDef 	  *gpioPort;
	uint16_t		  pin;
	bool 			  isInitialized;
	uint8_t 		  bitstream[MAX_BITS];
	uint16_t 		  bitstream_len;
} initStruct_t;

typedef enum{
	ERROR_PARAM = -1,
	PARAM_OK
} sendCanFrameStatus_t;

void delayUs(initStruct_t *init, uint32_t us);
sendCanFrameStatus_t initSoftwareCan(initStruct_t *init, TIM_HandleTypeDef *tim, GPIO_TypeDef* gpioPort, uint16_t GPIO_Pin);
sendCanFrameStatus_t sendCanFrame(initStruct_t *init, uint16_t id, uint8_t dlc, uint8_t *data);

#endif /* INC_SOFTWARECAN_H_ */
