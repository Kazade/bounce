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

#ifndef PROFILER_H
#define PROFILER_H

#include <bounce/common/math/math.h>
#include <bounce/common/memory/block_pool.h>
#include <bounce/common/template/array.h>
#include <bounce/common/time.h>

// Profiler node
struct ProfilerNode
{
	const char* name;
	float64 t0;
	float64 t1;
	ProfilerNode* parent;
	b3StackArray<ProfilerNode*, 32> children;
};

// A single-threaded profiler.
class Profiler
{
public:
	Profiler();
	
	~Profiler();

	// Must be called before profiling. 
	void Begin();

	// Must be called after profiling.
	void End();

	// Begin a new scope.
	void BeginScope(const char* name);
	
	// End the top scope.
	void EndScope();

	// Get the root profiler node.
	ProfilerNode* GetRoot() { return m_root; }
private:
	ProfilerNode* CreateNode();
	void DestroyNode(ProfilerNode* node);
	
	void RecurseDestroyNode(ProfilerNode* node);

	b3BlockPool m_pool; // pool of nodes
	b3Time m_time; // timer
	ProfilerNode* m_root; // tree root node
	ProfilerNode* m_top; // top node
};

extern Profiler* g_profiler;

#endif