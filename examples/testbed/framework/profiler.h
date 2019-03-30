/*
* Copyright (c) 2016-2019 Irlan Robson https://irlanrobson.github.io
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

#ifndef PROFILER_H
#define PROFILER_H

#include <bounce/common/math/math.h>
#include <bounce/common/time.h>
#include <bounce/common/template/array.h>

class ProfilerListener;

// A time-stamped profiler event.
struct ProfilerEvent
{
	const char* name;
	float64 t0;
	float64 t1;
	ProfilerEvent* parent;
};

// A single-threaded event-based profiler.
class Profiler
{
public:
	Profiler();
	
	~Profiler();

	// Must be called before profiling. 
	void Begin();

	// Must be called after profiling.
	// The function will report all events in this profiler 
	// to the given event listener in the correct calling order.
	// This function also flushes the profiler.
	void End();

	// Add a profiler event to the queue.
	// You can control the maximum number of profiler events using 
	// MAX_PROFILER_EVENTS.
	void PushEvent(const char* name);
	
	// Remove the top profiler event.
	void PopEvent();
private:
	b3Time m_time;
	b3StackArray<ProfilerEvent, 256> m_events;
	ProfilerEvent* m_top;
};

extern Profiler* g_profiler;

// Any implementation of this interface passed to Profiler::End will listen to profile events.
class ProfilerListener
{
public:
	virtual ~ProfilerListener() { }

	// This function is called when profiling has began.
	virtual void BeginEvents() { }

	// This function is called when profiling has ended.
	virtual void EndEvents() { }
	
	// This function is called when a profiler event begins.
	virtual void BeginEvent(const char* name, float64 time) 
	{
		B3_NOT_USED(name);
		B3_NOT_USED(time);
	}

	// This function is called when a profiler event ends.
	virtual void EndEvent(const char* name, float64 time)
	{
		B3_NOT_USED(name);
		B3_NOT_USED(time);
	}
};

extern ProfilerListener* g_profilerListener;

#endif