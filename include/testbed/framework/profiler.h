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
#include <bounce/common/template/array.h>

enum ProfileType
{
	e_begin,
	e_end
};

void ProfileBeginEvents();

void ProfileEvent(i32 tid, i32 pid, const char* name, float64 time, ProfileType type);
void ProfileEvent(i32 tid, i32 pid, const char* name, float64 elapsed);

void ProfileEndEvents();

void ProfileBegin();
void ProfileEnd();

struct ProfileRecord
{
	float64 elapsed;
	float64 maxElapsed;
	const char* name;
};

class Profiler
{
public:
	void Clear()
	{
		for (u32 i = 0; i < m_records.Count(); ++i)
		{
			m_records[i].elapsed = 0;
		}
	}

	void Add(const char* name, float64 elapsed)
	{
		for (u32 i = 0; i < m_records.Count(); ++i)
		{
			ProfileRecord& r = m_records[i];
			if (r.name == name)
			{
				r.elapsed += elapsed;
				r.maxElapsed = b3Max(r.maxElapsed, r.elapsed);
				return;
			}
		}

		ProfileRecord r;
		r.elapsed = elapsed;
		r.maxElapsed = 0;
		r.name = name;

		m_records.PushBack(r);
	}

	b3StackArray<ProfileRecord, 256> m_records;
};

#endif