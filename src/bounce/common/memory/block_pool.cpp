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

#include <bounce/common/memory/block_pool.h>

b3BlockPool::b3BlockPool(u32 blockSize)
{
	m_blockSize = blockSize;
	m_chunkSize = b3_blockCount * m_blockSize;

	m_chunks = NULL;
	m_chunkCount = 0;

	// Pre-allocate some chunks
	b3Chunk* chunk = (b3Chunk*)b3Alloc(sizeof(b3Chunk) + m_chunkSize);
	++m_chunkCount;
	chunk->freeBlocks = (b3Block*)((u8*)chunk + sizeof(b3Chunk));

#ifdef _DEBUG
	memset(chunk->freeBlocks, 0xcd, m_chunkSize);
#endif

	// Link the singly-linked list of the new block of the chunk.
	for (u32 i = 0; i < b3_blockCount - 1; ++i)
	{
		b3Block* current = (b3Block*)((u8*)chunk->freeBlocks + i * blockSize);
		current->next = (b3Block*)((u8*)chunk->freeBlocks + (i + 1) * blockSize);
	}
	b3Block* last = (b3Block*)((u8*)chunk->freeBlocks + (b3_blockCount - 1) * blockSize);
	last->next = NULL;

	// Push back the new chunk of the singly-linked list of chunks.
	chunk->next = m_chunks;
	m_chunks = chunk;
}

b3BlockPool::~b3BlockPool()
{
	b3Chunk* c = m_chunks;
	while (c)
	{
		b3Chunk* quack = c;
		c = c->next;
		b3Free(quack);
		--m_chunkCount;
	}
	B3_ASSERT(m_chunkCount == 0);
}

void* b3BlockPool::Allocate()
{
	if (m_chunks)
	{
		if (m_chunks->freeBlocks)
		{
			b3Block* block = m_chunks->freeBlocks;
			m_chunks->freeBlocks = block->next;
			return block;
		}
	}

	// Allocate a new chunk of memory.
	b3Chunk* chunk = (b3Chunk*)b3Alloc(sizeof(b3Chunk) + m_chunkSize);
	++m_chunkCount;
	chunk->freeBlocks = (b3Block*)((u8*)chunk + sizeof(b3Chunk));

#ifdef _DEBUG
	memset(chunk->freeBlocks, 0xcd, m_chunkSize);
#endif

	// Link the singly-linked list of the new block of the chunk.
	for (u32 i = 0; i < b3_blockCount - 1; ++i)
	{
		b3Block* current = (b3Block*)((u8*)chunk->freeBlocks + i * m_blockSize);
		current->next = (b3Block*)((u8*)chunk->freeBlocks + (i + 1) * m_blockSize);
	}
	b3Block* last = (b3Block*)((u8*)chunk->freeBlocks + (b3_blockCount - 1) * m_blockSize);
	last->next = NULL;

	// Push back the new chunk of the singly-linked list of chunks.
	chunk->next = m_chunks;
	m_chunks = chunk;

	// Make the free block of the chunk available for the next allocation.
	b3Block* block = m_chunks->freeBlocks;
	m_chunks->freeBlocks = block->next;
	return block;
}

void b3BlockPool::Free(void* p)
{
#ifdef _DEBUG
	// Verify the block was allocated from this allocator.
	bool found = false;
	b3Chunk* c = m_chunks;
	for (u32 i = 0; i < m_chunkCount; ++i)
	{
		b3Chunk* chunk = c;
		// Memory aabb test.
		b3Block* blocks = (b3Block*)((u8*)chunk + sizeof(b3Chunk));
		if ((u8*)blocks <= (u8*)p && (u8*)p + m_blockSize <= (u8*)blocks + m_chunkSize)
		{
			found = true;
			break;
		}
		c = c->next;
	}
	B3_ASSERT(found);
	memset(p, 0xfd, m_blockSize);
#endif

	b3Block* block = (b3Block*)p;
	block->next = m_chunks->freeBlocks;
	m_chunks->freeBlocks = block;
}
