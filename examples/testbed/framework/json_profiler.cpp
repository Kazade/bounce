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

#include <testbed/framework/json_profiler.h>

#define STRING(x) String(x, sizeof(x) - 1)

JsonProfiler::JsonProfiler()
{
	m_file = nullptr;
	m_stream = nullptr;
	m_writer = nullptr;
}

JsonProfiler::~JsonProfiler()
{

}

void JsonProfiler::BeginEvents()
{
	if (m_file)
	{
		return;
	}

	m_file = fopen("profile.json", "wt");
	if (!m_file)
	{
		return;
	}

	static char buffer[512];
	m_stream = new FileWriteStream(m_file, buffer, sizeof(buffer));

	m_writer = new Writer<FileWriteStream>(*m_stream);

	m_writer->StartObject();
	m_writer->STRING("traceEvents");
	m_writer->StartArray();
}

void JsonProfiler::EndEvents()
{
	if (!m_writer)
	{
		return;
	}

	m_writer->EndArray();
	m_writer->EndObject();

	delete m_writer;
	m_writer = nullptr;

	delete m_stream;
	m_stream = nullptr;

	fclose(m_file);
	m_file = nullptr;
}

void JsonProfiler::BeginEvent(i32 tid, i32 pid, const char* name, float64 t)
{
	if (!m_writer)
	{
		return;
	}

	const char* phase = "B";

	float64 scale = 1000.0;

	m_writer->StartObject();
	m_writer->STRING("pid");  m_writer->Int(pid);
	m_writer->STRING("tid");  m_writer->Int(tid);
	m_writer->STRING("ts");   m_writer->Int64((u64)(t * scale));
	m_writer->STRING("ph");   m_writer->String(phase, 1);
	m_writer->STRING("cat");  m_writer->STRING("physics");
	m_writer->STRING("name"); m_writer->String(name, strlen(name));
	m_writer->STRING("args"); m_writer->StartObject(); m_writer->EndObject();
	m_writer->EndObject();
}

void JsonProfiler::EndEvent(i32 tid, i32 pid, const char* name, float64 t)
{
	if (!m_writer)
	{
		return;
	}

	const char* phase = "E";

	float64 scale = 1000.0;

	m_writer->StartObject();
	m_writer->STRING("pid");  m_writer->Int(pid);
	m_writer->STRING("tid");  m_writer->Int(tid);
	m_writer->STRING("ts");   m_writer->Int64((u64)(t * scale));
	m_writer->STRING("ph");   m_writer->String(phase, 1);
	m_writer->STRING("cat");  m_writer->STRING("physics");
	m_writer->STRING("name"); m_writer->String(name, strlen(name));
	m_writer->STRING("args"); m_writer->StartObject(); m_writer->EndObject();
	m_writer->EndObject();
}

#undef STRING