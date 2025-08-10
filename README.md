This project is a software implementation of a CAN bus (125 kbit/s) using an STM32F072 (nucleo board) microcontroller. 

How to run:
1) Use a HSE external crystal. Required for stable bit generation.
2) Use a timer. I use TIM2. Set the timer to generate a pulse of 1 microsecond (needed for the delayUs() function to work properly).
3) Add a add a softwareCan.c and softwareCan.h into your workspace
4) Include a softwareCan.h file.
5) Run a timer and preape a data to send (payload and size).
6) In softwareCan.c file, paste a extern TIM_HandleTypeDef (needed for the delayUs() function to work properly).
7) In compiler setting, set a optimization level -O1.
8) Call a function sendCanFrame() with a parameters: CAN_ID (11bit), payload, size (max 8 byte).
9) Connect to a logic analazer to your CAN bit banding pin & enjoy!

WARNING: This is only a sample CAN bus project demonstrating bit-banding techniques. It is not intended for use in real-world applications!
