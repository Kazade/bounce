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

#ifndef PROFILER_RECORDER_H
#define PROFILER_RECORDER_H

#include <bounce/common/math/math.h>
#include <bounce/common/template/array.h>

struct ProfilerNode;

// A persistent profiler record
struct ProfilerRecord
{
	const char* name;
	float64 elapsed;
	float64 minElapsed;
	float64 maxElapsed;
	u32 callCount;
};

// This class maintains persistent profiler records
class ProfilerRecorder
{
public:
	void BuildRecords();
	
	void BuildSortedRecords(b3Array<ProfilerRecord*>& output);
	
	const b3Array<ProfilerRecord>& GetRecords() const { return m_records; }
private:
	void RecurseBuildRecords(ProfilerNode* node);
	
	void RecurseBuildSortedRecords(ProfilerNode* node, b3Array<ProfilerRecord*>& output);

	ProfilerRecord* FindRecord(const char* name);

	b3StackArray<ProfilerRecord, 256> m_records; // persistent profiler records
};

extern ProfilerRecorder* g_profilerRecorder;

#endif