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

#include <bounce/collision/broad_phase.h>

b3BroadPhase::b3BroadPhase() 
{
	m_moveBufferCapacity = 16;
	m_moveBuffer = (u32*)b3Alloc(m_moveBufferCapacity * sizeof(u32));
	memset(m_moveBuffer, 0, m_moveBufferCapacity * sizeof(u32));
	m_moveBufferCount = 0;

	m_pairCapacity = 16;
	m_pairs = (b3Pair*)b3Alloc(m_pairCapacity * sizeof(b3Pair));
	memset(m_pairs, 0, m_pairCapacity * sizeof(b3Pair));
	m_pairCount = 0;
}

b3BroadPhase::~b3BroadPhase() 
{
	b3Free(m_moveBuffer);
	b3Free(m_pairs);
}

void b3BroadPhase::BufferMove(u32 proxyId) 
{
	// The proxy has been moved. Add it to the buffer of moved proxies.
	// Check capacity.
	if (m_moveBufferCount == m_moveBufferCapacity) 
	{
		// Duplicate capacity.
		m_moveBufferCapacity *= 2;

		u32* oldMoveBuffer = m_moveBuffer;
		m_moveBuffer = (u32*)b3Alloc(m_moveBufferCapacity * sizeof(u32));
		memcpy(m_moveBuffer, oldMoveBuffer, m_moveBufferCount * sizeof(u32));
		b3Free(oldMoveBuffer);
	}

	// Add to move buffer.
	m_moveBuffer[m_moveBufferCount] = proxyId;
	++m_moveBufferCount;
}

bool b3BroadPhase::TestOverlap(u32 proxy1, u32 proxy2) const 
{
	return m_tree.TestOverlap(proxy1, proxy2);
}

u32 b3BroadPhase::CreateProxy(const b3AABB3& aabb, void* userData) 
{
	// Later, if the node aabb has changed then it should be reinserted into the tree.
	// However, this can be expansive due to the hierarchy reconstruction.
	// Therefore, the original AABB is extended and inserted into the tree,
	// so we can check later if the new (original) AABB is inside the old (fat) AABB.
	b3AABB3 fatAABB = aabb;
	fatAABB.Extend(B3_AABB_EXTENSION);	
	u32 proxyId = m_tree.InsertNode(fatAABB, userData);
	BufferMove(proxyId);
	return proxyId;
}

void b3BroadPhase::DestroyProxy(u32 proxyId) 
{
	return m_tree.RemoveNode(proxyId);
}

bool b3BroadPhase::MoveProxy(u32 proxyId, const b3AABB3& aabb, const b3Vec3& displacement)
{
	if (m_tree.GetAABB(proxyId).Contains(aabb))
	{
		// Do nothing if the new AABB is contained in the old AABB.
		return false;
	}

	// Update the tree with a fat and motion predicted AABB.
	const b3Vec3 kExtension(B3_AABB_EXTENSION, B3_AABB_EXTENSION, B3_AABB_EXTENSION);

	// Extend the new (original) AABB.
	b3AABB3 fatAABB;
	fatAABB.m_lower = aabb.m_lower - kExtension;
	fatAABB.m_upper = aabb.m_upper + kExtension;

	if (displacement.x < 0.0f)
	{
		fatAABB.m_lower.x += B3_AABB_MULTIPLIER * displacement.x;
	}
	else
	{
		fatAABB.m_upper.x += B3_AABB_MULTIPLIER * displacement.x;
	}

	if (displacement.y < 0.0f)
	{
		fatAABB.m_lower.y += B3_AABB_MULTIPLIER * displacement.y;
	}
	else
	{
		fatAABB.m_upper.y += B3_AABB_MULTIPLIER * displacement.y;
	}

	if (displacement.z < 0.0f)
	{
		fatAABB.m_lower.z += B3_AABB_MULTIPLIER * displacement.z;
	}
	else
	{
		fatAABB.m_upper.z += B3_AABB_MULTIPLIER * displacement.z;
	}

	// Update proxy with the extented AABB.
	m_tree.UpdateNode(proxyId, fatAABB);
	
	// Buffer the moved proxy.
	BufferMove(proxyId);
	
	// Notify the proxy has moved.
	return true;
}

bool b3BroadPhase::Report(u32 proxyId) 
{
	if (proxyId == m_queryProxyId) 
	{
		// The proxy can't overlap with itself.
		return true;
	}

	// Check capacity.
	if (m_pairCount == m_pairCapacity) 
	{
		// Duplicate capacity.
		m_pairCapacity *= 2;
		
		b3Pair* oldPairs = m_pairs;
		m_pairs = (b3Pair*)b3Alloc(m_pairCapacity * sizeof(b3Pair));
		memcpy(m_pairs, oldPairs, m_pairCount * sizeof(b3Pair));
		b3Free(oldPairs);
	}

	// Add overlapping pair to the pair buffer.
	m_pairs[m_pairCount].proxy1 = b3Min(proxyId, m_queryProxyId);
	m_pairs[m_pairCount].proxy2 = b3Max(proxyId, m_queryProxyId);
	++m_pairCount;

	// Keep looking for overlapping pairs.
	return true;
}