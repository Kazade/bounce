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

#ifndef TESTBED_PROFILER_H
#define TESTBED_PROFILER_H

#include <testbed/framework/profiler.h>
#include <testbed/framework/recorder_profiler.h>

// Set to 1 then the testbed profiler will write profile events into a .json file.
// Set to 0 otherwise.
#define PROFILE_JSON 0

#if (PROFILE_JSON == 1)
	#include <testbed\framework\json_profiler.h>
#endif

class TestbedProfiler : public ProfilerListener
{
public:
	void BeginEvents() override
	{
		m_recorderListener.BeginEvents();

#if (PROFILE_JSON == 1)
		m_jsonListener.BeginEvents();
#endif

	}

	void EndEvents() override
	{
		m_recorderListener.EndEvents();

#if (PROFILE_JSON == 1)
		m_jsonListener.EndEvents();
#endif

	}

	void BeginEvent(const char* name, float64 time) override
	{
		m_recorderListener.BeginEvent(name, time);

#if (PROFILE_JSON == 1)
		m_jsonListener.BeginEvent(name, time);
#endif
	}

	void EndEvent(const char* name, float64 time) override
	{
		m_recorderListener.EndEvent(name, time);

#if (PROFILE_JSON == 1)
		m_jsonListener.EndEvent(name, time);
#endif
	}

	RecorderProfiler m_recorderListener;

#if (PROFILE_JSON == 1)
	JsonProfiler m_jsonListener;
#endif

};

#endif