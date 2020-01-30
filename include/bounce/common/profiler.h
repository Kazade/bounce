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

#ifndef B3_PROFILER_H
#define B3_PROFILER_H

#include <bounce/common/math/math.h>
#include <bounce/common/memory/block_pool.h>
#include <bounce/common/time.h>

// Profiler node statistics
struct b3ProfilerNodeStat
{
	const char* name;
	scalar64 minElapsed;
	scalar64 maxElapsed;
	b3ProfilerNodeStat* next;
};

// A profiler node
struct b3ProfilerNode
{
	const char* name; 
	scalar64 t0;
	scalar64 t1; 
	
	scalar64 elapsed; // total elapsed time
	u32 callCount; // number of calls inside the parent node
	u32 callCountRec; // used for detecting recursive calls

	b3ProfilerNode* parent; // parent node
	b3ProfilerNode* head; // list of children
	b3ProfilerNode* next; // link to the next node in the parent node list of children

	b3ProfilerNodeStat* stat; // global node statistics
};

// A hierarchical profiler
class b3Profiler
{
public:
	b3Profiler();
	
	~b3Profiler();

	// Must be called before profiling. 
	void Begin();

	// Must be called after profiling.
	void End();

	// Begin a new scope.
	void BeginScope(const char* name);

	// End the top scope.
	void EndScope();

	// Get the root node.
	b3ProfilerNode* GetRoot() { return m_root; }
	
	// Get the top node.
	b3ProfilerNode* GetTop() { return m_top; }
	
	// Get the list of node statistics.
	b3ProfilerNodeStat* GetStats() const { return m_stats; }
private:
	b3ProfilerNode* CreateNode();
	void DestroyNode(b3ProfilerNode* node);
	
	void RecurseDestroyNode(b3ProfilerNode* node);

	b3ProfilerNode* FindNode(const char* name);

	b3ProfilerNodeStat* CreateStat();

	b3ProfilerNodeStat* FindStat(const char* name);

	b3BlockPool m_nodePool; // pool of nodes
	b3BlockPool m_statPool; // pool of node stats
	b3Time m_time; // timer
	b3ProfilerNode* m_root; // tree root node
	b3ProfilerNode* m_top; // top node
	b3ProfilerNodeStat* m_stats; // node stats
};

#endif