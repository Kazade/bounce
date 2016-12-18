/*
* Copyright (c) 2016-2016 Irlan Robson http://www.irlan.net
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include <bounce\common\time.h>

// This file contains platform dependent code 
// and may not compile depending of the OS.

#if (_WIN32 == 1)

#include <Windows.h>

float64 GetCyclesPerSecond()
{
	LARGE_INTEGER integer;
	QueryPerformanceFrequency(&integer);
	return float64(integer.QuadPart);
}

u64 GetCycleCount()
{
	LARGE_INTEGER integer;
	QueryPerformanceCounter(&integer);
	return integer.QuadPart;
}

#endif

float64 b3Time::m_invFrequency = 0.0;

b3Time::b3Time()
{
	m_lastTime = 0;
	m_curTime = 0;

	if (m_invFrequency == 0.0)
	{
		float64 cyclesPerSec = GetCyclesPerSecond();
		float64 secPerCycles = 1.0 / cyclesPerSec;
		float64 milisecPerCycles = 1000.0 * secPerCycles;
		m_invFrequency = milisecPerCycles;
	}

	m_lastRealTime = GetCycleCount();
}

void b3Time::Update() 
{
	// Retrieve the current time in units of cycles.
	u64 curTime = GetCycleCount();
	u64 dt = curTime - m_lastRealTime;
	m_lastRealTime = curTime;
		
	float64 dtMs = m_invFrequency * float64(dt);
	
	UpdateBy(dtMs);
}

void b3Time::UpdateBy(float64 delta)
{
	m_lastTime = m_curTime;
	m_curTime += delta;
}
