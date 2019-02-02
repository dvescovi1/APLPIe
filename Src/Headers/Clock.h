#pragma once

#include <stdint.h>
#include "Peripheral.h"


//The following is undocumented :( Taken from http://www.scribd.com/doc/127599939/BCM2835-Audio-clocks
//each write to CM_PWMTL and CM_PWMDIV requires the password to be written:
#define PWMCTL_PASSWD 0x5a000000
#define PWMDIV_PASSWD 0x5a000000

#define PWMCTL_MASH(x) (((x)&0x3) << 9)
#define PWMCTL_MASH0 PWMTRL_MASH(0)
#define PWMCTL_MASH1 PWMTRL_MASH(1)
#define PWMCTL_MASH2 PWMTRL_MASH(2)
#define PWMCTL_MASH3 PWMTRL_MASH(3)
#define PWMCTL_FLIP (1<<8) //use to inverse clock polarity
#define PWMCTL_BUSY (1<<7) //read-only flag that indicates clock generator is running.
#define PWMCTL_KILL (1<<5) //write a 1 to stop & reset clock generator. USED FOR DEBUG ONLY
#define PWMCTL_ENAB (1<<4) //gracefully stop/start clock generator. BUSY flag will go low once clock is off.
#define PWMCTL_SRC(x) ((x)&0xf) //clock source. 0=gnd. 1=oscillator. 2-3=debug. 4=PLLA per. 5=PLLC per. 6=PLLD per. 7=HDMI aux. 8-15=GND
#define PWMCTL_SRC_OSC PWMCTL_SRC(1)
#define PWMCTL_SRC_PLLA PWMCTL_SRC(4)
#define PWMCTL_SRC_PLLC PWMCTL_SRC(5)
#define PWMCTL_SRC_PLLD PWMCTL_SRC(6)

//max clock divisor is 4095
#define PWMDIV_DIVI(x) (((x)&0xfff) << 12)
#define PWMDIV_DIVF(x) ((x)&0xfff)

struct ClockRegisters
{
	uint32_t Unknown[38];
	uint32_t PCMCTL;
	uint32_t PCMDIV;
	uint32_t PWMCTL;
	uint32_t PWMDIV;
};

typedef struct
{
	union
	{
		PeripheralInfo info;
		volatile ClockRegisters* Base;
	};
} ClockInfo;

class Clock : public Peripheral
{
private:
	ClockInfo _clock;

public:
	Clock(const char* name);
	void virtual SysInit();
	void virtual SysUninit();

	void Disable();
	void Enable();
	void SetDivider(int divider);

	void TestExample();
};
