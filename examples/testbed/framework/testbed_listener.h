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

#ifndef TESTBED_LISTENER_H
#define TESTBED_LISTENER_H

#include <testbed\framework\profiler.h>
#include <testbed\framework\recorder_profiler.h>

// Set to 1 then the testbed listener will write profile events into a .json file.
// Set to 0 otherwise.
#define PROFILE_JSON 0

#if (PROFILE_JSON == 1)
	#include <testbed\framework\json_profiler.h>
#endif


class TestbedListener : public ProfilerListener
{
public:
	void BeginEvents() override
	{
		m_recorderProfiler.BeginEvents();

#if (PROFILE_JSON == 1)
		m_jsonListener.BeginEvents();
#endif

	}

	void EndEvents() override
	{
		m_recorderProfiler.EndEvents();

#if (PROFILE_JSON == 1)
		m_jsonListener.EndEvents();
#endif

	}

	void BeginEvent(i32 tid, i32 pid, const char* name, float64 time) override
	{
#if (PROFILE_JSON == 1)
		m_jsonListener.BeginEvent(tid, pid, name, time);
#endif

	}

	void EndEvent(i32 tid, i32 pid, const char* name, float64 time) override
	{
#if (PROFILE_JSON == 1)
		m_jsonListener.EndEvent(tid, pid, name, time);
#endif

	}

	void Duration(const char* name, float64 time) override
	{
		m_recorderProfiler.Add(name, time);
	}

	RecorderProfiler m_recorderProfiler;

#if (PROFILE_JSON == 1)
	JsonProfiler m_jsonListener;
#endif

};

#endif