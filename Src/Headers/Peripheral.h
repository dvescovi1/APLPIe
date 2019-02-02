#pragma once

#include <stdint.h>

struct PeripheralInfo
{	
	// This MUST be first.  Intent is for specific periperhals to
	// create a union where this location points to peripheral
	// specific register maps.
	volatile uint32_t* MappedAddress;
	uint32_t* BaseAddress;
	int lengthBytes;
};

class Peripheral
{
private:
	static int _fdMem;
	
	const char* _name;

	static void DelayMicrosecondsHard(uint32_t howLong);

protected:
	static int _pageSize;

	void Map(int addr, int length, PeripheralInfo& info);
	void Unmap(PeripheralInfo& info);
	void WriteBit(volatile uint32_t *dest, uint32_t mask, uint32_t value);
	
public:
	Peripheral(const char* name);
	virtual void SysInit() = 0;
	virtual void SysUninit() = 0;

	static void DelayMicroseconds(uint32_t howLong);
	static void DelayMilliseconds(uint32_t howLong);	
};
