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

#ifndef JSON_PROFILER_H
#define JSON_PROFILER_H

#include <bounce/common/settings.h>

#include <rapidjson/filewritestream.h>
#include <rapidjson/writer.h>

using namespace rapidjson;

// The following profiler listener is notified by a profiler when events are initiated 
// or terminated.
// When it receives the notification it immediately saves its data into a .json file format.
// The .json file can be read and interpreted by the Google Chrome Tracing.
// Say chrome://tracing to the web browser and load the file
// This file is by default called "profile.json". Any name can be given. 
// For implementation details, see json_profile.cpp.
class JsonProfiler
{
public:
	JsonProfiler();
	~JsonProfiler();

	void BeginEvents();

	void EndEvents();

	void BeginEvent(i32 tid, i32 pid, const char* name, float64 time);

	void EndEvent(i32 tid, i32 pid, const char* name, float64 time);
private:
	FILE * m_file;
	FileWriteStream* m_stream;
	Writer<FileWriteStream>* m_writer;
};

#endif