/*
 * PulseGenerator.h:
 *	Another Peripheral Library for the raspberry PI.
 *	Copyright (c) 2019 Alger Pike
 ***********************************************************************
 * This file is part of APLPIe:
 *	https://github.com/AlgerP572/APLPIe
 *
 *    APLPIe is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    APLPIe is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with APLPIe.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */
#pragma once

#include "Device.h"
#include "Gpio.h"

#include <vector>

struct Pulse
{
	PinState State;
	uint32_t Duration; // Duration is defined by the clocks current clock frequency
};

struct PulseTrain
{
	uint32_t Pin; // Pin or pins to output the pulse train on.
	std::vector<Pulse> Timing;
};


class PulseGenerator : public Device
{
private:
	Gpio& _gpio;
	std::vector<PulseTrain> _pulseTracks;

public:
	PulseGenerator(Gpio& gpio);
	void AddPulseTrain(PulseTrain& pulseTrain);
};