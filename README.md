## Non-blocking I2C driver for TM1650 based 7-segment LED display driver on STM32 Nucleo

Hi! There's a demo driver for TM1650-based to a STM32 Nucleo

Since I couldn't find any existing drivers, I decided to implement my own. The main goal was to write a code that allows to show digits on the screen through interrupts and learn I2C protocol;

I used TM1650's datasheet and figured out my self how to display the minus "-" sign which wasn't included in the datasheet. 


The driver is now capable of displaying numbers of different type in the range:
(0 : 9999) - unsigned four-digit integer number
(-999 : 999) - signed three-digit integer
(-99.9 : 99.9) - signed float, 1 decimal place
(-9.99 : 9.99) - signed float, 2 decimal places

It uses I2C protocol and needs a pull-up resistors. 

Here's the video showing how the display work.
One decimal place float demo:

https://github.com/user-attachments/assets/34a2b795-784f-4ddf-b88a-0235775e0cfe

Brightness demo (it was done to check if brightness can be changed smoothly - it was based on basic HAL_Delay() so the displayed number changes gradually and slowly).

https://github.com/user-attachments/assets/09879d9c-7bda-4ede-be97-98b434fcadd4

I'll also attach the photo of wiring to STM32 Nucleo Board - a schematic will be added in the next release.
![Wiring](/Images%20and%20videos/wiring.jpeg)
