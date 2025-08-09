/*
 * softwareCan.c
 *
 *  Created on: Aug 9, 2025
 *      Author: cezar
 */

#include "main.h"
#include "softwareCan.h"

#define CAN_ONE_BIT_TIME_US	8
#define MAX_BITS         	256

/*add own timmer instance*/
extern TIM_HandleTypeDef htim2;
/*-----------------------------*/

static uint8_t bitstream[MAX_BITS];
static int bitstream_len = 0;

static void gpioCanWriteBit(uint8_t bit);
static void prepareCanFrame(uint16_t id, uint8_t dlc, uint8_t *data);
static void appendBit(uint8_t *buf, int *len, uint8_t bit, uint8_t *last, int *count);
static uint16_t calculateCanCrc(const uint8_t *bits, int bit_len);

/*Public fn*/
void delayUs(uint32_t us) /*mandatory use HSE!*/
{
	uint16_t start = __HAL_TIM_GET_COUNTER(&htim2);
	while ((__HAL_TIM_GET_COUNTER(&htim2) - start) < us);
}

void sendCanFrame(uint16_t id, uint8_t dlc, uint8_t *data)
{
    prepareCanFrame(id, dlc, data);

    for (int i = 0; i < bitstream_len; i++)
    {
        gpioCanWriteBit(bitstream[i]);
    }
}

/*Private fn*/
static void gpioCanWriteBit(uint8_t bit)
{
    if (bit)
    	CAN_GPIO_Port->BSRR = CAN_Pin;
    	//HAL_GPIO_WritePin(CAN_GPIO_Port, CAN_Pin, 1); /*too long execute!*/
    else
    	CAN_GPIO_Port->BRR = CAN_Pin;
    	//HAL_GPIO_WritePin(CAN_GPIO_Port, CAN_Pin, 0);

    delayUs(CAN_ONE_BIT_TIME_US);
}

static void appendBit(uint8_t *buf, int *len, uint8_t bit, uint8_t *last, int *count)
{
    buf[(*len)++] = bit;

    if (bit == *last)
    {
        (*count)++;

        if (*count == 5)
        {
            buf[(*len)++] = !bit; // Stuff bit
            *count = 1;
            *last = !bit;
        }
    }
    else
    {
        *count = 1;
    }

    *last = bit;
}

/*Calculate a CAN CRC15*/
static uint16_t calculateCanCrc(const uint8_t *bits, int bit_len)
{
    uint16_t crc = 0;
    const uint16_t poly = 0x4599;

    for (int i = 0; i < bit_len; ++i)
    {
        uint8_t bit = bits[i];
        uint8_t crc_msb = (crc >> 14) & 1;
        crc <<= 1;

        if (bit ^ crc_msb)
        {
            crc ^= poly;
        }
    }

    return crc & 0x7FFF;
}

static void prepareCanFrame(uint16_t id, uint8_t dlc, uint8_t *data)
{
    bitstream_len = 0;
    int count = 1;
    uint8_t last = 0;

    uint8_t crc_input[128];
    int crc_bit_len = 0;

    // SOF
    appendBit(bitstream, &bitstream_len, 0, &last, &count);

    // 11-bit ID
    for (int i = 10; i >= 0; i--) {
        uint8_t b = (id >> i) & 1;
        crc_input[crc_bit_len++] = b;
        appendBit(bitstream, &bitstream_len, b, &last, &count);
    }

    // RTR = 0
    crc_input[crc_bit_len++] = 0;
    appendBit(bitstream, &bitstream_len, 0, &last, &count);
    // IDE = 0
    crc_input[crc_bit_len++] = 0;
    appendBit(bitstream, &bitstream_len, 0, &last, &count);
    // r0 = 0
    crc_input[crc_bit_len++] = 0;
    appendBit(bitstream, &bitstream_len, 0, &last, &count);

    // DLC (4 bity)
    for (int i = 3; i >= 0; i--)
    {
        uint8_t b = (dlc >> i) & 1;

        crc_input[crc_bit_len++] = b;
        appendBit(bitstream, &bitstream_len, b, &last, &count);
    }

    // DATA
    for (int i = 0; i < dlc; i++)
    {
        for (int b = 7; b >= 0; b--)
        {
            uint8_t bit = (data[i] >> b) & 1;
            crc_input[crc_bit_len++] = bit;
            appendBit(bitstream, &bitstream_len, bit, &last, &count);
        }
    }

    // CRC-15
    uint16_t crc = calculateCanCrc(crc_input, crc_bit_len);
    for (int i = 14; i >= 0; i--)
    {
        uint8_t b = (crc >> i) & 1;
        appendBit(bitstream, &bitstream_len, b, &last, &count);
    }

    // CRC delimiter
    appendBit(bitstream, &bitstream_len, 1, &last, &count);

    // ACK slot + delimiter
    appendBit(bitstream, &bitstream_len, 0, &last, &count);
    appendBit(bitstream, &bitstream_len, 1, &last, &count);
}
