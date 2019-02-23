# APLPIe
Another Peripheral Library for raspberry Pi.

![Alt text](Imgs/applepie.jpg?raw=true "Title")

This library is a set of C++ classes meant to be compiled
into your application.  The base classes allow access
to the raspberry pi peripherals and some basic application
support functionality like predefined init and uninit.

Many ideas and some code snippets taken from:

WiringPi:
https ://projects.drogon.net/raspberry-pi/wiringpi/

Pigpio:
http://abyz.me.uk/rpi/pigpio/

Raspberry-Pi-DMA-Example
https://github.com/Wallacoloo/Raspberry-Pi-DMA-Example

Happy Coding!

## GPIO
The APLPIe library supports hi-speed direct user mode GPIO pin
read and writes. Currently the mode of access is via mapped
memory which requires sudo level access.  Although, access can
also be achieved via the linux gpio driver, other peripherals
in this library such as user mode Dma also require sudo access.

The following code illustrates the bit bang test displayed in the 
trace:

>   '// Bit bang test...'
>	'for (int i = 0; i < 100; i++)'
>	'{'
>		
>	'	gpio.WritePin(DmaPin1, PinState::High);'
>	'	gpio.WritePin(DmaPin1, PinState::Low);'
>	'}'
	
![Alt text](Imgs/BitBangTest.png?raw=true "BitBangTest")