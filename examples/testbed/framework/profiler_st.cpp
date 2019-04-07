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

#include <testbed/framework/profiler_st.h>

ProfilerSt* g_profilerSt = nullptr;

ProfilerSt::ProfilerSt() : m_pool(sizeof(ProfilerStNode))
{
	m_root = nullptr;
	m_top = nullptr;
}

ProfilerSt::~ProfilerSt()
{
	assert(m_root == nullptr);
	assert(m_top == nullptr);
}

ProfilerStNodeStat* ProfilerSt::FindStat(const char* name)
{
	for (u32 i = 0; i < m_stats.Count(); ++i)
	{
		if (m_stats[i].name == name)
		{
			return &m_stats[i];
		}
	}

	return nullptr;
}

ProfilerStNodeStat* ProfilerSt::CreateStat()
{
	m_stats.PushBack(ProfilerStNodeStat());
	return &m_stats.Back();
}

ProfilerStNode* ProfilerSt::CreateNode()
{
	void* block = m_pool.Allocate();
	return new (block) ProfilerStNode();
}

void ProfilerSt::DestroyNode(ProfilerStNode* node)
{
	node->~ProfilerStNode();
	m_pool.Free(node);
}

void ProfilerSt::RecurseDestroyNode(ProfilerStNode* node)
{
	for (u32 i = 0; i < node->children.Count(); ++i)
	{
		return RecurseDestroyNode(node->children[i]);
	}

	DestroyNode(node);
}

static ProfilerStNode* RecurseFindNode(ProfilerStNode* node, const char* name)
{
	if (node->name == name)
	{
		return node;
	}

	ProfilerStNode* result = nullptr;
	for (u32 i = 0; result == nullptr && i < node->children.Count(); ++i)
	{
		result = RecurseFindNode(node->children[i], name);
	}

	return result;
}

ProfilerStNode* ProfilerSt::FindNode(const char* name)
{
	if (m_top)
	{
		return RecurseFindNode(m_top, name);
	}

	if (m_root)
	{
		return RecurseFindNode(m_root, name);
	}

	return nullptr;
}

void ProfilerSt::BeginScope(const char* name)
{
	ProfilerStNode* fn = FindNode(name);

	if (fn)
	{
		m_time.Update();
		fn->t0 = m_time.GetCurrentMilis();
		++fn->callCount;
		m_top = fn;
		return;
	}

	m_time.Update();

	ProfilerStNode* n = CreateNode();
	n->name = name;
	n->t0 = m_time.GetCurrentMilis();
	n->elapsed = 0.0f;
	n->callCount = 1;
	n->parent = m_top;
	n->stat = nullptr;

	if (m_root == nullptr)
	{
		m_root = n;
		m_top = n;

		return;
	}

	if (m_top)
	{
		m_top->children.PushBack(n);
	}

	m_top = n;
}

void ProfilerSt::EndScope()
{
	assert(m_top != nullptr);

	m_time.Update();

	m_top->t1 = m_time.GetCurrentMilis();
	
	float64 elapsedTime = m_top->t1 - m_top->t0;

	m_top->elapsed += elapsedTime;
	
	ProfilerStNodeStat* stat = FindStat(m_top->name);

	if (stat == nullptr)
	{
		stat = CreateStat();
		stat->name = m_top->name;
		stat->minElapsed = elapsedTime;
		stat->maxElapsed = elapsedTime;	
	}
	else
	{
		stat->minElapsed = b3Min(stat->minElapsed, elapsedTime);
		stat->maxElapsed = b3Max(stat->maxElapsed, elapsedTime);
	}

	if (m_top->stat == nullptr)
	{
		m_top->stat = stat;
	}

	assert(m_top->stat == stat);

	m_top = m_top->parent;
}

void ProfilerSt::Begin()
{
	assert(m_top == nullptr);
}

void ProfilerSt::End()
{
	assert(m_top == nullptr);
	
	if (m_root)
	{
		RecurseDestroyNode(m_root);
		m_root = nullptr;
	}
}