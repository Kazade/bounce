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

#ifndef PROFILER_ST_H
#define PROFILER_ST_H

#include <bounce/common/math/math.h>
#include <bounce/common/memory/block_pool.h>
#include <bounce/common/template/array.h>
#include <bounce/common/time.h>

// Profiler tree node statistics
struct ProfilerStNodeStat
{
	const char* name;
	float64 minElapsed;
	float64 maxElapsed;
};

// A profiler tree node
struct ProfilerStNode
{
	const char* name; 
	float64 t0;
	float64 t1; 
	
	float64 elapsed;
	u32 callCount;
	
	ProfilerStNode* parent;
	b3StackArray<ProfilerStNode*, 32> children;

	ProfilerStNodeStat* stat;
};

// A profiler tree 
class ProfilerSt
{
public:
	ProfilerSt();
	
	~ProfilerSt();

	// Must be called before profiling. 
	void Begin();

	// Must be called after profiling.
	void End();

	// Begin a new scope.
	void BeginScope(const char* name);

	// End the top scope.
	void EndScope();

	ProfilerStNode* GetRoot() { return m_root; }
private:
	ProfilerStNode* CreateNode();
	void DestroyNode(ProfilerStNode* node);
	
	void RecurseDestroyNode(ProfilerStNode* node);

	ProfilerStNode* FindNode(const char* name);

	ProfilerStNodeStat* CreateStat();

	ProfilerStNodeStat* FindStat(const char* name);

	b3BlockPool m_pool; // pool of nodes
	b3Time m_time; // timer
	ProfilerStNode* m_root; // tree root node
	ProfilerStNode* m_top; // top node

	b3StackArray<ProfilerStNodeStat, 256> m_stats; // node statistics
};

extern ProfilerSt* g_profilerSt;

#endif