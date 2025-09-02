## TM1650 7- segment LED display driver for STM32 Nucleo

Hi! There's a demo driver for TM1650-based to a STM32 Nucleo



Since I couldn't find any existing drivers, I decided to implement my own. The main goal was to write a code that allows to show digits on the screen through interrupts and learn I2C protocol;

I used TM1650's datasheet and figured out my self how to display the minus "-" sign which wasn't included in the datasheet. 


The driver is capable of displaying integers in the range(-999:999);
It uses I2C protocol and needs a pull-up resistors. 

Although it does the job, it was my first driver and one of the ways to learn I2C protocol. It is not guaranteed to be fully robust I plan improve it soon.


Here's the video showing how the display work. I used ADC in 6-bit mode (0:63) and added a negative offset (-32)  to ADC's value in order to demonstrate negative numbers


https://github.com/user-attachments/assets/7be48017-775a-4a0e-8b86-e3b8694ca1c5
