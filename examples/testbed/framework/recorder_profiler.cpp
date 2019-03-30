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

#include <testbed/framework/recorder_profiler.h>

RecorderProfiler* g_profilerRecorder = nullptr;

void RecorderProfiler::BeginEvents()
{
	m_call = 0;
	for (u32 i = 0; i < m_records.Count(); ++i)
	{
		m_records[i].elapsed = 0.0;
		m_records[i].call = 0;
	}
}

void RecorderProfiler::EndEvents()
{
	for (u32 i = 0; i < m_records.Count(); ++i)
	{
		ProfilerRecord& r1 = m_records[i];

		for (u32 j = i + 1; j < m_records.Count(); ++j)
		{
			ProfilerRecord& r2 = m_records[j];

			if (r2.call < r1.call)
			{
				b3Swap(r1, r2);
			}
		}
	}
}

ProfilerRecord* RecorderProfiler::FindRecord(const char* name)
{
	for (u32 i = 0; i < m_records.Count(); ++i)
	{
		ProfilerRecord& r = m_records[i];
		if (r.name == name)
		{
			return &r;
		}
	}
	return nullptr;
}

void RecorderProfiler::Add(const char* name, float64 elapsedTime)
{
	B3_ASSERT(elapsedTime >= 0.0);

	++m_call;

	ProfilerRecord* fr = FindRecord(name);
	if (fr)
	{
		fr->elapsed += elapsedTime;
		fr->maxElapsed = b3Max(fr->maxElapsed, elapsedTime);
		fr->call = m_call;

		return;
	}

	ProfilerRecord r;
	r.name = name;
	r.elapsed = elapsedTime;
	r.maxElapsed = elapsedTime;
	r.call = m_call;

	m_records.PushBack(r);
}