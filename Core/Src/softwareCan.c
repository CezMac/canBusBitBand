/*
 * softwareCan.c
 *
 *  Created on: Aug 9, 2025
 *      Author: cezar
 */

#include "main.h"
#include "softwareCan.h"

/*USER CONFIG*/
#define SIMULATION_MODE 1
/*-----------------------------*/

#define CAN_ONE_BIT_TIME_US	8 /*125 kbit/s*/

static void delayUs(initStruct_t *init, uint32_t us);
static void gpioCanWriteBit(initStruct_t *init, uint8_t bit);
static void prepareCanFrame(initStruct_t *init, uint16_t id, uint8_t dlc, uint8_t *data);
static void appendBit(uint8_t *buf, uint16_t *len, uint8_t bit, uint8_t *last, int *count);
static uint16_t calculateCanCrc(const uint8_t *bits, int bit_len);

/*Public fn*/
/**
 * @brief Initializes the software CAN handle structure.
 *
 * This function sets up the CAN handle with the provided timer handle,
 * GPIO port, and GPIO pin. The timer is typically configured to run at
 * 1 MHz for accurate microsecond delays. Each handle represents an
 * independent CAN interface instance.
 *
 * @param[in,out] init     Pointer to the CAN handle structure to initialize
 * @param[in]     tim      Pointer to the timer handle used for timing operations
 * @param[in]     gpioPort Pointer to the GPIO port used for CAN signal control
 * @param[in]     GPIO_Pin GPIO pin number used for CAN signal output
 *
 * @retval PARAM_OK    Handle was initialized successfully
 * @retval ERROR_PARAM One or more parameters are invalid (NULL pointers or zero pin)
 *
 * @note This function must be called before using any other CAN functions
 *       with this handle instance.
 * @warning The timer must be pre-configured and running before calling this function.
 *
 * @example
 * @code
 * static initStruct_t canHandle = {0};
 * sendCanFrameStatus_t status = initSoftwareCan(&canHandle, &htim2, GPIOA, GPIO_PIN_5);
 * if (status != PARAM_OK) {
 *     // Handle initialization error
 * }
 * @endcode
 */
sendCanFrameStatus_t initSoftwareCan(initStruct_t *init, TIM_HandleTypeDef *tim, GPIO_TypeDef *gpioPort, uint16_t GPIO_Pin)
{
	if (tim == NULL || gpioPort == NULL || !GPIO_Pin)
		return ERROR_PARAM;

	init->tim = tim;
	init->gpioPort = gpioPort;
	init->pin = GPIO_Pin;
	init->isInitialized = true;

	return PARAM_OK;
}

/**
 * @brief Transmits a CAN frame bit-by-bit over a GPIO-based CAN interface.
 *
 * This function constructs and transmits a standard CAN 2.0A data frame
 * with the specified identifier, data length, and payload. The frame is
 * sent bit-by-bit using GPIO bit-banging with precise timing control.
 * The function handles CRC calculation, bit stuffing, and proper CAN
 * frame formatting automatically.
 *
 * @param[in,out] init Pointer to initialized CAN handle structure
 * @param[in]     id   CAN identifier (11-bit standard format, range: 0x000-0x7FF)
 * @param[in]     dlc  Data Length Code - number of data bytes (range: 0-8)
 * @param[in]     data Pointer to data buffer containing payload bytes
 *                     (must not be NULL, even if dlc=0)
 *
 * @retval PARAM_OK    CAN frame transmitted successfully
 * @retval ERROR_PARAM Invalid parameters:
 *                     - id > 0x7FF (exceeds 11-bit limit)
 *                     - dlc > 8 (exceeds maximum data length)
 *                     - data == NULL (invalid data pointer)
 *                     - Handle not initialized
 *
 * @note This function is blocking and will take approximately
 *       (64 + dlc*8) * 8µs to complete for a typical frame.
 * @warning Interrupts should be disabled during transmission to ensure
 *          accurate bit timing.
 * @warning This function modifies the internal bitstream buffer of the handle.
 *
 * @example
 * @code
 * static initStruct_t canHandle = {0};
 * uint8_t txData[] = {0xAA, 0xBB, 0xCC, 0xDD};
 *
 * // Initialize handle first
 * initSoftwareCan(&canHandle, &htim2, GPIOA, GPIO_PIN_5);
 *
 * // Send CAN frame with ID 0x123 and 4 bytes of data
 * sendCanFrameStatus_t status = sendCanFrame(&canHandle, 0x123, 4, txData);
 * if (status != PARAM_OK) {
 *     // Handle transmission error
 * }
 * @endcode
 *
 * @see initSoftwareCan() must be called before using this function
 * @see prepareCanFrame() for internal frame construction details
 */
sendCanFrameStatus_t sendCanFrame(initStruct_t *init, uint16_t id, uint8_t dlc, uint8_t *data)
{
	if(id > 0x7FF || dlc > 8 || data == NULL || init->isInitialized == false)
		return ERROR_PARAM;

	prepareCanFrame(init, id, dlc, data);

	for (int i = 0; i < init->bitstream_len; i++)
		gpioCanWriteBit(init, init->bitstream[i]);

	return PARAM_OK;
}

/*Private fn*/
void delayUs(initStruct_t *init, uint32_t us) /*mandatory use HSE!*/
{
	uint16_t start = __HAL_TIM_GET_COUNTER(init->tim);

	while ((__HAL_TIM_GET_COUNTER(init->tim) - start) < us);
}

static void gpioCanWriteBit(initStruct_t *init, uint8_t bit)
{
    if (bit)
    	init->gpioPort->BSRR = init->pin;
    	//CAN_GPIO_Port->BSRR = CAN_Pin;
    	//HAL_GPIO_WritePin(CAN_GPIO_Port, CAN_Pin, 1); /*too long execute!*/
    else
    	init->gpioPort->BRR = init->pin;
    	//CAN_GPIO_Port->BRR = CAN_Pin;
    	//HAL_GPIO_WritePin(CAN_GPIO_Port, CAN_Pin, 0);

    delayUs(init, CAN_ONE_BIT_TIME_US);
}

static void appendBit(uint8_t *buf, uint16_t *len, uint8_t bit, uint8_t *last, int *count)
{
    buf[(*len)++] = bit;

    if (bit == *last)
    {
        (*count)++;

        if (*count == 5) //add a stuff bit 5b/6b
        {
            uint8_t stuff = !bit;

            buf[(*len)++] = stuff; // Stuff bit
            *count = 0;
            *last = stuff;
        }
    }
    else
    {
        *count = 1;
        *last = bit;
    }
}

/*Calculate a CAN CRC15*/
static uint16_t calculateCanCrc(const uint8_t *bits, int bit_len)
{
    uint16_t crc = 0;
    const uint16_t poly = 0x4599; /*Polymian from CAN Bus standart*/

    for (int i = 0; i < bit_len; ++i)
    {
        uint8_t bit = bits[i];
        uint8_t crc_msb = (crc >> 14) & 1;

        crc <<= 1;

        if (bit ^ crc_msb)
            crc ^= poly;
    }

    return crc & 0x7FFF;
}

static void prepareCanFrame(initStruct_t *init, uint16_t id, uint8_t dlc, uint8_t *data)
{
	init->bitstream_len = 0;

    int count = 1;
    int crc_bit_len = 0;

    uint8_t last = 0;
    uint8_t crc_input[255];

    // SOF
    appendBit(init->bitstream, &init->bitstream_len, 0, &last, &count);

    // 11-bit ID
    for (int i = 10; i >= 0; i--)
    {
        uint8_t b = (id >> i) & 1;

        crc_input[crc_bit_len++] = b;
        appendBit(init->bitstream, &init->bitstream_len, b, &last, &count);
    }

    // RTR = 0
    crc_input[crc_bit_len++] = 0;
    appendBit(init->bitstream, &init->bitstream_len, 0, &last, &count);
    // IDE = 0
    crc_input[crc_bit_len++] = 0;
    appendBit(init->bitstream, &init->bitstream_len, 0, &last, &count);
    // r0 = 0
    crc_input[crc_bit_len++] = 0;
    appendBit(init->bitstream, &init->bitstream_len, 0, &last, &count);

    // DLC (4 bits)
    for (int i = 3; i >= 0; i--)
    {
        uint8_t b = (dlc >> i) & 1;

        crc_input[crc_bit_len++] = b;
        appendBit(init->bitstream, &init->bitstream_len, b, &last, &count);
    }

    // DATA
    for (int i = 0; i < dlc; i++)
    {
        for (int b = 7; b >= 0; b--)
        {
            uint8_t bit = (data[i] >> b) & 1;
            crc_input[crc_bit_len++] = bit;
            appendBit(init->bitstream, &init->bitstream_len, bit, &last, &count);
        }
    }

    // CRC-15
    uint16_t crc = calculateCanCrc(crc_input, crc_bit_len);

    for (int i = 14; i >= 0; i--)
    {
        uint8_t b = (crc >> i) & 1;
        appendBit(init->bitstream, &init->bitstream_len, b, &last, &count);
    }

    // CRC delimiter
    appendBit(init->bitstream, &init->bitstream_len, 1, &last, &count);

#if SIMULATION_MODE
    // ACK slot + delimiter
    appendBit(init->bitstream, &init->bitstream_len, 0, &last, &count);
    appendBit(init->bitstream, &init->bitstream_len, 1, &last, &count);
#endif

}
