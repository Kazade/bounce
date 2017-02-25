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

#include <testbed/framework/profiler.h>
#include <bounce/common/profiler.h>
#include <bounce/common/time.h>
#include <bounce/common/template/queue.h>

struct Event
{
	i32 tid;
	i32 pid;
	const char* name;
	float64 t0;
	float64 t1;
	Event* parent;
};

static b3Time time;
static b3BoundedQueue<Event, 256> events;
static Event* top = NULL;

bool b3PushProfileScope(const char* name)
{
	time.Update();

	Event e;
	e.tid = -1;
	e.pid = -1;
	e.t0 = time.GetCurrentMilis();
	e.t1 = 0;
	e.name = name;
	e.parent = top;

	Event* back = events.Push(e);
	if (back)
	{
		top = back;
	}

	return back != NULL;
}

void b3PopProfileScope()
{
	B3_ASSERT(top);
	B3_ASSERT(top->t1 == 0);

	time.Update();
	top->t1 = time.GetCurrentMilis();
	B3_ASSERT(top->t1 != 0);
	top = top->parent;
}

void ProfileBegin()
{
	B3_ASSERT(events.IsEmpty());
}

void ProfileEnd()
{
	ProfileBeginEvents();

	while (events.IsEmpty() == false)
	{
		const Event& e = events.Front();
		events.Pop();

		ProfileEvent(e.tid, e.pid, e.name, e.t0, e_begin);
		ProfileEvent(e.tid, e.pid, e.name, e.t1, e_end);
		ProfileEvent(e.tid, e.pid, e.name, e.t1 - e.t0);
	}

	B3_ASSERT(events.IsEmpty());

	ProfileEndEvents();
}

//

#define PROFILER_SCREEN 1
#define PROFILER_JSON 2

#define PROFILER_OUTPUT PROFILER_SCREEN

#if PROFILER_OUTPUT == PROFILER_SCREEN

extern Profiler* g_profiler;

void ProfileBeginEvents()
{

}

void ProfileEvent(i32 tid, i32 pid, const char* name, float64 time, ProfileType type)
{

}

void ProfileEvent(i32 tid, i32 pid, const char* name, float64 elapsed)
{
	g_profiler->Add(name, elapsed);
}

void ProfileEndEvents()
{

}

#elif PROFILER_OUTPUT == PROFILER_JSON

#include <rapidjson/filewritestream.h>
#include <rapidjson/writer.h>

using namespace rapidjson;

static FILE* file = NULL;
static FileWriteStream* stream = NULL;
static Writer<FileWriteStream>* writer = NULL;

#define STRING(x) String(x, sizeof(x) - 1)

void ProfileBeginEvents()
{
	if (file)
	{
		return;
	}

	file = fopen("profile.json", "wt");
	if (!file)
	{
		return;
	}

	static char buffer[512];
	stream = new FileWriteStream(file, buffer, sizeof(buffer));

	writer = new Writer<FileWriteStream>(*stream);

	writer->StartObject();
	writer->STRING("traceEvents");
	writer->StartArray();
}

void ProfileEvent(i32 tid, i32 pid, const char* name, float64 time, ProfileType type)
{
	if (!writer)
	{
		return;
	}

	const char* phase = 0;
	switch (type)
	{
	case ProfileType::e_begin:  phase = "B"; break;
	case ProfileType::e_end:    phase = "E"; break;
	default: B3_ASSERT(false);
	}

	float64 scale = 1000000;

	writer->StartObject();
	writer->STRING("pid");  writer->Int(pid);
	writer->STRING("tid");  writer->Int(tid);
	writer->STRING("ts");   writer->Int64((u64)(time * scale));
	writer->STRING("ph");   writer->String(phase, 1);
	writer->STRING("cat");  writer->STRING("physics");
	writer->STRING("name"); writer->String(name, strlen(name));
	writer->STRING("args"); writer->StartObject(); writer->EndObject();
	writer->EndObject();
}

void ProfileEvent(i32 tid, i32 pid, const char* name, float64 elapsed)
{
}

void ProfileEndEvents()
{
	if (!writer)
	{
		return;
	}

	writer->EndArray();
	writer->EndObject();

	delete writer;
	writer = NULL;

	delete stream;
	stream = NULL;

	fclose(file);
	file = NULL;
}

#undef STRING

#else

#endif