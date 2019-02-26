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

#ifndef RECORDER_PROFILER_H
#define RECORDER_PROFILER_H

#include <bounce/common/template/array.h>
#include <bounce/common/math/math.h>

// An event in the profiler event recorder.
struct ProfilerRecord
{
	float64 elapsed;
	float64 maxElapsed;
	const char* name;
	u32 call;
};

// The profiler recorder simply keeps profile events in an event buffer, 
// so that they can be rendered, saved to a file, etc. later in a 
// particular position in code.
class RecorderProfiler
{
public:
	void BeginEvents();

	void EndEvents();

	void Add(const char* name, float64 elapsedTime);

	const b3Array<ProfilerRecord>& GetRecords() const { return m_records; }
private:
	b3StackArray<ProfilerRecord, 256> m_records;
	u32 m_count;
};

extern RecorderProfiler* g_profilerRecorder;

#endif