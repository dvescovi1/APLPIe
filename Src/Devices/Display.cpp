#include "../Headers/Display.h"

#include "../Headers/Delay.h"
#include "../Headers/Peripheral.h"
#include "../Headers/Gpio.h"

// Contains the code to actual the pins for the four digit 7 segment display.
// These are the ideal Segment codes as if pins 0-7 were used to display the
// characters...
// During constuction a new map is created that is specific to the pins that
// your harware isactually using.
uint32_t const FourDigitSevenSegmentDisplay::unmappedSegCode[10] = { 0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f };

FourDigitSevenSegmentDisplay::FourDigitSevenSegmentDisplay(Gpio* gpio,
	int pin0,
	int pin1,
	int pin2,
	int pin3,
	CharacterDisplayPins* characterPins) :
	Device("LedDisplay")
{
	_gpio = gpio;
	_pin0 = pin0;
	_pin1 = pin1;
	_pin2 = pin2;
	_pin3 = pin3;
	_characterPins = characterPins;

	RemapSegCodes();
}

void FourDigitSevenSegmentDisplay::RemapSegCodes()
{
	for (int j = 0; j < 10; j++)
	{
		for (int i = 0; i < 8; i++)
		{
			uint32_t segCode = unmappedSegCode[j] & (1 << i);

			if (segCode > 0)
			{
				mappedSegCode[j] |= (1 << _characterPins->_charPins[i]);
			}
			else
			{
				mappedSegCode[j] &= ~(1 << _characterPins->_charPins[i]);
			}
		}
	}
}

void FourDigitSevenSegmentDisplay::SysInit(void)
{
	_gpio->SetPinMode(_pin0, PinMode::Output);
	_gpio->SetPinMode(_pin1, PinMode::Output);
	_gpio->SetPinMode(_pin2, PinMode::Output);
	_gpio->SetPinMode(_pin3, PinMode::Output);

	_gpio->WritePin(_pin0, PinState::High);
	_gpio->WritePin(_pin1, PinState::High);
	_gpio->WritePin(_pin2, PinState::High);
	_gpio->WritePin(_pin3, PinState::High);
	
	for (int i = 0; i < 8; i++)
	{
		_gpio->SetPinMode(_characterPins->_charPins[i], PinMode::Output);
		_gpio->WritePin(_characterPins->_charPins[i], PinState::High);
	}
}

void FourDigitSevenSegmentDisplay::SysUninit(void)
{

}

void FourDigitSevenSegmentDisplay::SetDisplayValue(int value)
{
	_datBuf[0] = mappedSegCode[value % 10];
	_datBuf[1] = mappedSegCode[value % 100 / 10];
	_datBuf[2] = mappedSegCode[value % 1000 / 100];
	_datBuf[3] = mappedSegCode[value / 1000];
}

void FourDigitSevenSegmentDisplay::Display(void)
{
	_gpio->WritePin(_pin0, PinState::Low);
	_gpio->WritePin(_pin1, PinState::High);
	_gpio->WritePin(_pin2, PinState::High);
	_gpio->WritePin(_pin3, PinState::High);
	_gpio->WritePins031(_characterPins->_characterMask, _datBuf[0]);

	Delay::Microseconds(50);

	_gpio->WritePin(_pin0, PinState::High);
	_gpio->WritePin(_pin1, PinState::Low);
	_gpio->WritePin(_pin2, PinState::High);
	_gpio->WritePin(_pin3, PinState::High);
	_gpio->WritePins031(_characterPins->_characterMask, _datBuf[1]);

	Delay::Microseconds(50);

	_gpio->WritePin(_pin0, PinState::High);
	_gpio->WritePin(_pin1, PinState::High);
	_gpio->WritePin(_pin2, PinState::Low);
	_gpio->WritePin(_pin3, PinState::High);
	_gpio->WritePins031(_characterPins->_characterMask, _datBuf[2]);

	Delay::Microseconds(50);

	_gpio->WritePin(_pin0, PinState::High);
	_gpio->WritePin(_pin1, PinState::High);
	_gpio->WritePin(_pin2, PinState::High);
	_gpio->WritePin(_pin3, PinState::Low);
	_gpio->WritePins031(_characterPins->_characterMask, _datBuf[3]);

	Delay::Microseconds(50);
}