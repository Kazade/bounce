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

#include <testbed/framework/profiler.h>

Profiler* g_profiler = nullptr;
ProfilerListener* g_profilerListener = nullptr;

Profiler::Profiler() : m_pool(sizeof(ProfilerNode))
{
	m_root = nullptr;
	m_top = nullptr;
}

Profiler::~Profiler()
{
	assert(m_root == nullptr);
	assert(m_top == nullptr);
}

ProfilerNode* Profiler::CreateNode()
{
	void* block = m_pool.Allocate();
	ProfilerNode* n = new (block) ProfilerNode();
	return n;
}

void Profiler::DestroyNode(ProfilerNode* node)
{
	node->~ProfilerNode();
	m_pool.Free(node);
}

void Profiler::BeginScope(const char* name)
{
	m_time.Update();

	ProfilerNode* n = CreateNode();
	n->name = name;
	n->t0 = m_time.GetCurrentMilis();
	n->t1 = 0.0;
	n->parent = m_top;

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

void Profiler::EndScope()
{
	m_time.Update();
	
	assert(m_top != nullptr);
	m_top->t1 = m_time.GetCurrentMilis();
	assert(m_top->t1 > m_top->t0);

	m_top = m_top->parent;
}

void Profiler::Begin()
{
	// If this assert is hit then it means Profiler::End hasn't been called.
	assert(m_root == nullptr);
	assert(m_top == nullptr);
}

static inline void RecurseEvents(ProfilerNode* node)
{
	ProfilerListener* listener = g_profilerListener;

	if (listener)
	{
		listener->BeginEvent(node->name, node->t0);

		listener->EndEvent(node->name, node->t1);
	}

	for (u32 i = 0; i < node->children.Count(); ++i)
	{
		RecurseEvents(node->children[i]);
	}
}

void Profiler::RecurseDestroyNode(ProfilerNode* node)
{
	for (u32 i = 0; i < node->children.Count(); ++i)
	{
		RecurseDestroyNode(node->children[i]);
	}

	DestroyNode(node);
}

void Profiler::End()
{
	assert(m_top == nullptr);
	
	ProfilerListener* listener = g_profilerListener;

	if (listener)
	{
		listener->BeginEvents();
	}

	RecurseEvents(m_root);
	
	RecurseDestroyNode(m_root);
	m_root = nullptr;

	assert(m_root == nullptr);

	if (listener)
	{
		listener->EndEvents();
	}
}