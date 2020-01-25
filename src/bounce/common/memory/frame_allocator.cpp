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

#include <bounce/common/memory/frame_allocator.h>

b3FrameAllocator::b3FrameAllocator()
{
	m_p = m_memory;
	m_allocatedSize = 0;
}

b3FrameAllocator::~b3FrameAllocator()
{
}

void* b3FrameAllocator::Allocate(u32 size)
{
	u32 totalSize = sizeof(b3Block) + size;

	if (m_allocatedSize + totalSize > b3_maxFrameSize)
	{
		u8* p = (u8*)b3Alloc(totalSize);
		
		b3Block* block = (b3Block*)p;
		block->p = p;
		block->size = size;
		block->parent = true;
		return p + sizeof(b3Block);
	}

	u8* p = m_p;
	m_p += totalSize;
	m_allocatedSize += totalSize;
	
	b3Block* block = (b3Block*)p;
	block->p = p;
	block->size = size;
	block->parent = false;
	return p + sizeof(b3Block);
}

void b3FrameAllocator::Free(void* q)
{
	u8* p = (u8*)(q) - sizeof(b3Block);
	b3Block* block = (b3Block*)p;
	B3_ASSERT(block->p == p);
	if (block->parent)
	{
		b3Free(p);
	}
}

void b3FrameAllocator::Reset()
{
	m_p = m_memory;
	m_allocatedSize = 0;
}