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
#include <bounce/common/memory/block_pool.h>
#include <bounce/common/template/array.h>
#include <bounce/common/time.h>

class ProfilerListener;

// Profiler node
struct ProfilerNode
{
	const char* name;
	float64 t0;
	float64 t1;
	ProfilerNode* parent;
	b3StackArray<ProfilerNode*, 32> children;
};

// A single-threaded profiler.
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

	// Push an event.
	void PushEvent(const char* name);
	
	// Remove the top profiler event.
	void PopEvent();
private:
	ProfilerNode* CreateNode();
	void DestroyNode(ProfilerNode* node);
	
	void RecurseDestroy(ProfilerNode* node);

	b3BlockPool m_pool; // pool of nodes
	b3Time m_time; // timer
	ProfilerNode* m_root; // tree root node
	ProfilerNode* m_top; // top node
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