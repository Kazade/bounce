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

#include <testbed/framework/profiler_recorder.h>
#include <testbed/framework/profiler.h>

ProfilerRecorder* g_profilerRecorder = nullptr;

ProfilerRecord* ProfilerRecorder::FindRecord(const char* name)
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

void ProfilerRecorder::RecurseBuildRecords(ProfilerNode* node)
{
	ProfilerRecord* fr = FindRecord(node->name);
	if (fr)
	{
		float64 elapsedTime = node->t1 - node->t0;

		fr->elapsed += elapsedTime;
		fr->minElapsed = b3Min(fr->minElapsed, elapsedTime);
		fr->maxElapsed = b3Max(fr->maxElapsed, elapsedTime);
		++fr->callCount;
	}
	else
	{
		float64 elapsedTime = node->t1 - node->t0;

		ProfilerRecord r;
		r.name = node->name;
		r.elapsed = elapsedTime;
		r.minElapsed = elapsedTime;
		r.maxElapsed = elapsedTime;
		r.callCount = 1;

		m_records.PushBack(r);
	}
	
	for (u32 i = 0; i < node->children.Count(); ++i)
	{
		RecurseBuildRecords(node->children[i]);
	}
}

void ProfilerRecorder::BuildRecords()
{
	for (u32 i = 0; i < m_records.Count(); ++i)
	{
		m_records[i].elapsed = 0.0;
		m_records[i].callCount = 0;
	}

	RecurseBuildRecords(g_profiler->m_root);
}

static ProfilerRecord* FindSortedRecord(b3Array<ProfilerRecord*>& records, const char* name)
{
	for (u32 i = 0; i < records.Count(); ++i)
	{
		ProfilerRecord* r = records[i];
		if (r->name == name)
		{
			return r;
		}
	}
	return nullptr;
}

void ProfilerRecorder::RecurseBuildSortedRecords(ProfilerNode* node, b3Array<ProfilerRecord*>& output)
{
	ProfilerRecord* fsr = FindSortedRecord(output, node->name);
	
	if (fsr == nullptr)
	{
		ProfilerRecord* fr = FindRecord(node->name);
	
		assert(fr != nullptr);

		// Push back the first ocurrence of call in calling order
		output.PushBack(fr);
	}
	
	for (u32 i = 0; i < node->children.Count(); ++i)
	{
		RecurseBuildSortedRecords(node->children[i], output);
	}	
}

void ProfilerRecorder::BuildSortedRecords(b3Array<ProfilerRecord*>& output)
{
	assert(output.Count() == 0);
	
	output.Reserve(m_records.Count());
	
	RecurseBuildSortedRecords(g_profiler->m_root, output);
}