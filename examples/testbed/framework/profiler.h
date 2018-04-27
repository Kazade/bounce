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

#ifndef PROFILER_H
#define PROFILER_H

#include <bounce/common/math/math.h>
#include <bounce/common/time.h>
#include <bounce/common/template/queue.h>
#include <bounce/common/template/array.h>

// This defines the maximum number of profiler events that can be 
// queued per frame until the function Profiler::Flush is called.
#define MAX_PROFILER_EVENTS 256

class ProfilerListener;

// A time-stamped profiler event.
struct ProfilerEvent
{
	i32 tid;
	i32 pid;
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
	void End(ProfilerListener* listener);

	// Add a profiler event to the queue.
	// Return true if the even has been added to the event queue 
	// or false if the queue is full.
	// You can control the maximum number of profiler events using 
	// MAX_PROFILER_EVENTS.
	bool PushEvent(const char* name);
	
	// Remove the top profiler event.
	void PopEvent();
private:
	b3Time m_time;
	b3BoundedQueue<ProfilerEvent, MAX_PROFILER_EVENTS> m_events;
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
	virtual void BeginEvent(i32 tid, i32 pid, const char* name, float64 time) 
	{
		B3_NOT_USED(tid);
		B3_NOT_USED(pid);
		B3_NOT_USED(name);
		B3_NOT_USED(time);
	}

	// This function is called when a profiler event ends.
	virtual void EndEvent(i32 tid, i32 pid, const char* name, float64 time)
	{
		B3_NOT_USED(tid);
		B3_NOT_USED(pid);
		B3_NOT_USED(name);
		B3_NOT_USED(time);
	}

	// This function is called when a profiler event ends.
	// However it supplies the duration of the last begin and end events.
	virtual void Duration(const char* name, float64 duration)
	{
		B3_NOT_USED(name);
		B3_NOT_USED(duration);
	}
};

extern ProfilerListener* g_profilerListener;

#endif