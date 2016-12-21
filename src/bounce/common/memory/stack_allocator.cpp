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

#include <bounce/common/memory/stack_allocator.h>

b3StackAllocator::b3StackAllocator() 
{
	m_blockCapacity = 256;
	m_blocks = (b3Block*)b3Alloc(m_blockCapacity * sizeof(b3Block));
	m_blockCount = 0;
	m_allocatedSize = 0;
}

b3StackAllocator::~b3StackAllocator() 
{
	B3_ASSERT(m_allocatedSize == 0);
	B3_ASSERT(m_blockCount == 0);
	b3Free(m_blocks);
}

void* b3StackAllocator::Allocate(u32 size) 
{
	if (m_blockCount == m_blockCapacity) 
	{
		// Then duplicate capacity if needed.
		b3Block* oldBlocks = m_blocks;
		m_blockCapacity *= 2;
		m_blocks = (b3Block*)b3Alloc(m_blockCapacity * sizeof(b3Block));
		memcpy(m_blocks, oldBlocks, m_blockCount * sizeof(b3Block));
		b3Free(oldBlocks);
	}

	b3Block* block = m_blocks + m_blockCount;
	block->size = size;
	if (m_allocatedSize + size > b3_maxStackSize) 
	{
		// Allocate with parent allocator.
		block->data = (u8*) b3Alloc(size);
		block->parent = true;
	}
	else 
	{
		// Use this stack memory.
		block->data = m_memory + m_allocatedSize;
		block->parent = false;
		m_allocatedSize += size;
	}
	
	++m_blockCount;

	return block->data;
}

void b3StackAllocator::Free(void* p) 
{
	B3_ASSERT(m_blockCount > 0);
	b3Block* block = m_blocks + m_blockCount - 1;
	B3_ASSERT(block->data == p);
	if (block->parent) 
	{
		b3Free(p);
	}
	else 
	{
		m_allocatedSize -= block->size;
	}
	--m_blockCount;
}
