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

static b3Time s_time;
static b3BoundedQueue<Event, 256> s_events;
static Event* s_top = NULL;

bool b3PushProfileScope(const char* name)
{
	s_time.Update();

	Event e;
	e.tid = -1;
	e.pid = -1;
	e.t0 = s_time.GetCurrentMilis();
	e.t1 = 0;
	e.name = name;
	e.parent = s_top;

	Event* back = s_events.Push(e);
	if (back)
	{
		s_top = back;
	}

	return back != NULL;
}

void b3PopProfileScope()
{
	B3_ASSERT(s_top);
	B3_ASSERT(s_top->t1 == 0);

	s_time.Update();
	s_top->t1 = s_time.GetCurrentMilis();
	B3_ASSERT(s_top->t1 != 0);
	s_top = s_top->parent;
}

void ProfileBegin()
{
	B3_ASSERT(s_events.IsEmpty());
}

void ProfileEnd()
{
	ProfileBeginEvents();

	while (s_events.IsEmpty() == false)
	{
		const Event& e = s_events.Front();
		s_events.Pop();

		ProfileEvent(e.tid, e.pid, e.name, e.t0, e_begin);
		ProfileEvent(e.tid, e.pid, e.name, e.t1, e_end);
		ProfileEvent(e.tid, e.pid, e.name, e.t1 - e.t0);
	}

	B3_ASSERT(s_events.IsEmpty());

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

void ProfileEvent(i32 tid, i32 pid, const char* name, float64 t, ProfileType type)
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

static FILE* s_file = NULL;
static FileWriteStream* s_stream = NULL;
static Writer<FileWriteStream>* s_writer = NULL;

#define STRING(x) String(x, sizeof(x) - 1)

void ProfileBeginEvents()
{
	if (s_file)
	{
		return;
	}

	s_file = fopen("profile.json", "wt");
	if (!s_file)
	{
		return;
	}

	static char buffer[512];
	s_stream = new FileWriteStream(s_file, buffer, sizeof(buffer));

	s_writer = new Writer<FileWriteStream>(*s_stream);

	s_writer->StartObject();
	s_writer->STRING("traceEvents");
	s_writer->StartArray();
}

void ProfileEvent(i32 tid, i32 pid, const char* name, float64 t, ProfileType type)
{
	if (!s_writer)
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

	float64 scale = 1000.0;

	s_writer->StartObject();
	s_writer->STRING("pid");  s_writer->Int(pid);
	s_writer->STRING("tid");  s_writer->Int(tid);
	s_writer->STRING("ts");   s_writer->Int64((u64)(t * scale));
	s_writer->STRING("ph");   s_writer->String(phase, 1);
	s_writer->STRING("cat");  s_writer->STRING("physics");
	s_writer->STRING("name"); s_writer->String(name, strlen(name));
	s_writer->STRING("args"); s_writer->StartObject(); s_writer->EndObject();
	s_writer->EndObject();
}

void ProfileEvent(i32 tid, i32 pid, const char* name, float64 elapsed)
{
}

void ProfileEndEvents()
{
	if (!s_writer)
	{
		return;
	}

	s_writer->EndArray();
	s_writer->EndObject();

	delete s_writer;
	s_writer = NULL;

	delete s_stream;
	s_stream = NULL;

	fclose(s_file);
	s_file = NULL;
}

#undef STRING

#else

#endif