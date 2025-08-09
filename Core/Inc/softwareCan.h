/*
 * softwareCan.h
 *
 *  Created on: Aug 9, 2025
 *      Author: cezar
 */

#ifndef INC_SOFTWARECAN_H_
#define INC_SOFTWARECAN_H_

void delay_us(uint32_t us);
void send_can_frame(uint16_t id, uint8_t dlc, uint8_t *data);

#endif /* INC_SOFTWARECAN_H_ */
