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

#include <bounce/common/profiler.h>

b3Profiler::b3Profiler() :
	m_nodePool(sizeof(b3ProfilerNode)),
	m_statPool(sizeof(b3ProfilerNodeStat))
{
	m_root = nullptr;
	m_top = nullptr;
	m_stats = nullptr;
}

b3Profiler::~b3Profiler()
{
	B3_ASSERT(m_root == nullptr);
	B3_ASSERT(m_top == nullptr);

	b3ProfilerNodeStat* s = m_stats;
	while (s)
	{
		b3ProfilerNodeStat* boom = s;
		s = s->next;
		boom->~b3ProfilerNodeStat();
		m_statPool.Free(boom);
	}
}

b3ProfilerNodeStat* b3Profiler::FindStat(const char* name)
{
	b3ProfilerNodeStat* s = m_stats;
	while (s)
	{
		if (s->name == name)
		{
			return s;
		}
		s = s->next;
	}

	return nullptr;
}

b3ProfilerNodeStat* b3Profiler::CreateStat()
{
	void* block = m_statPool.Allocate();
	return new (block) b3ProfilerNodeStat();
}

b3ProfilerNode* b3Profiler::CreateNode()
{
	void* block = m_nodePool.Allocate();
	return new (block) b3ProfilerNode();
}

void b3Profiler::DestroyNode(b3ProfilerNode* node)
{
	node->~b3ProfilerNode();
	m_nodePool.Free(node);
}

void b3Profiler::RecurseDestroyNode(b3ProfilerNode* node)
{
	b3ProfilerNode* n = node->head;
	while (n)
	{
		b3ProfilerNode* boom = n;
		n = n->next;
		RecurseDestroyNode(boom);
	}

	DestroyNode(node);
}

b3ProfilerNode* b3Profiler::FindNode(const char* name)
{
	if (m_top)
	{
		b3ProfilerNode* n = m_top->head;
		while (n)
		{
			if (n->name == name)
			{
				return n;
			}
			n = n->next;
		}
	}

	return nullptr;
}

void b3Profiler::BeginScope(const char* name)
{
	b3ProfilerNode* fn = FindNode(name);

	if (fn)
	{
		m_top = fn;
		++fn->callCount;
		
		if (fn->callCountRec == 0)
		{
			m_time.Update();
			fn->t0 = m_time.GetCurrentMilis();
		}

		++fn->callCountRec;
		return;
	}

	b3ProfilerNode* n = CreateNode();
	n->name = name;
	m_time.Update();
	n->t0 = m_time.GetCurrentMilis();
	n->elapsed = scalar64(0);
	n->callCount = 1;
	n->callCountRec = 1;
	n->stat = nullptr;
	n->parent = m_top;
	n->head = nullptr;
	n->next = nullptr;

	if (m_root == nullptr)
	{
		m_root = n;
		m_top = n;

		return;
	}

	if (m_top)
	{
		n->next = m_top->head;
		m_top->head = n;
	}

	m_top = n;
}

void b3Profiler::EndScope()
{
	B3_ASSERT(m_top != nullptr);

	--m_top->callCountRec;
	if (m_top->callCountRec > 0)
	{
		return;
	}

	m_time.Update();
	m_top->t1 = m_time.GetCurrentMilis();

	scalar64 elapsedTime = m_top->t1 - m_top->t0;

	m_top->elapsed += elapsedTime;

	b3ProfilerNodeStat* stat = FindStat(m_top->name);

	if (stat == nullptr)
	{
		stat = CreateStat();
		stat->name = m_top->name;
		stat->minElapsed = elapsedTime;
		stat->maxElapsed = elapsedTime;
		stat->next = m_stats;
		m_stats = stat;
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

	B3_ASSERT(m_top->stat == stat);

	m_top = m_top->parent;
}

void b3Profiler::Begin()
{
	B3_ASSERT(m_top == nullptr);
}

void b3Profiler::End()
{
	B3_ASSERT(m_top == nullptr);

	if (m_root)
	{
		RecurseDestroyNode(m_root);
		m_root = nullptr;
	}
}