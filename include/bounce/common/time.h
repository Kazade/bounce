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

#ifndef B3_TIME_H
#define B3_TIME_H

#include <bounce\common\settings.h>

// A timer class that accumulates time.
// Usefull for measuring elapsed times of 
// sections of code.
class b3Time 
{
public :
	b3Time();
	
	// Get the accumulated time in miliseconds
	// from this timer.
	float64 GetCurMilis() const;

	// Get the elapsed time since this timer was updated.
	float64 GetElapsedMilis() const;

	// Add the elapsed time since this function
	// was called to this timer.
	void Update();

	// Add a given ammout of time to this timer.
	void UpdateBy(float64 dt);
private:
	static float64 m_invFrequency;
	
	u64 m_lastRealTime;
	float64 m_lastTime;
	float64 m_curTime;
};

inline float64 b3Time::GetCurMilis() const 
{ 
	return m_curTime;
}

inline float64 b3Time::GetElapsedMilis() const 
{ 
	return m_curTime - m_lastTime;
}

#endif
