APLPIe
======

Another Peripheral Library for raspberry Pi.

![Alt text](media/d54388e3771ab9dd9dd31ef8bd238273.jpg)

Alt text

This library is a set of C++ classes meant to be compiled into your application.
The base classes allow access to the raspberry pi peripherals and some basic
application support functionality like predefined init and uninit.

Many ideas and some code snippets taken from:

WiringPi: https ://projects.drogon.net/raspberry-pi/wiringpi/

Pigpio: http://abyz.me.uk/rpi/pigpio/

Raspberry-Pi-DMA-Example https://github.com/Wallacoloo/Raspberry-Pi-DMA-Example

Happy Coding!

GPIO
----

The APLPIe library supports hi-speed direct user mode GPIO pin read and writes.
Currently the mode of access is via mapped memory which requires sudo level
access. Although, access can also be achieved via the linux gpio driver, other
peripherals in this library such as user mode Dma also require sudo access.

The following code illustrates the bit bang test displayed in the trace:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Bit bang test...
for (int i = 0; i < 100; i++)
{
    
    gpio.WritePin(DmaPin1, PinState::High);
    gpio.WritePin(DmaPin1, PinState::Low);
}
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

![Alt text](media/3457d4d6b2658ec627bcaa6a867d2f04.png)

Alt text

### void Gpio::Export(int pin);

Although this library uses the memory mapped peripheral address to write and
read the GPIO peripheral, exporting and Unexporting of the raspberry PI I/O pins
is still required in the user-land application. This suggests that the linux
GPIO kernel driver still has master control of the memory space and can override
and prevent the access of the GPIO memory addresses. To export a pin simply call
the export function with the number of the pin you would like to export. All
pins in the library are currently BCM only. (The ones printed on the circuit
board when looking at the breakout board.)

![Raspberry PI breakout board. This library uses the BCM (white printing) pin numbers](media/04c1d83907ecda6836d1fc5e199af7b6.jpg)

Raspberry PI breakout board. This library uses the BCM (white printing) pin
numbers

### void Gpio::Unexport(int pin)

When the user-land application is shutdown the GPIO pins should be restored to
their original un-exported state.

### void Gpio::SetPinMode(int pin, PinMode mode)

Once the GPIO pins have been exported they may be accessed by the memory mapped
base address. This function provides access to the pin mode register and allow
the following pin modes to be set:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// GPIO Function Select bits

enum class PinMode

{

Input = 0b000,

Output = 0b001,

Alt0 = 0b100,

Alt1 = 0b101,

Alt2 = 0b110,

Alt3 = 0b111,

Alt4 = 0b011,

Alt5 = 0b010

};

uint32_t fSel = gpioToGPFSEL[pin];

uint32_t shift = gpioToShift[pin];

volatile uint32_t\* address = \&Base-\>GPFSEL[fSel];

\*address \|= ((uint32_t) mode \<\< shift);

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

### void Gpio::SetPudMode(int pin, PudMode mode)

If the pin is configured as an output you must additionally specify whether it
should be a pull-up or pull-down pin. The reader is referred to the BCM
peripheral manual for the Broadcom chip. Currently this function supports the
correct timing sequence specified in the manual. The following modes are
supported:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

enum class PudMode

{

Off = 0b00, // disable pull - up / down

PullDown = 0b01, // Enable Pull Down control

PullUp = 0b10, // Enable Pull Up control

Reserved = 0b11

};

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
