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

#ifndef B3_STATIC_TREE_H
#define B3_STATIC_TREE_H

#include <bounce/common/template/stack.h>
#include <bounce/collision/shapes/aabb3.h>
#include <bounce/collision/collision.h>

#define B3_NULL_NODE_S (0xFFFFFFFF)

// AABB tree for static AABBs.
class b3StaticTree 
{
public:
	b3StaticTree();
	~b3StaticTree();

	// Build this tree from a list of AABBs.
	void Build(const b3AABB3* aabbs, u32 count);

	// Get the AABB of a given proxy.
	const b3AABB3& GetAABB(u32 proxyId) const;

	// Get the user data associated with a given proxy.
	u32 GetUserData(u32 proxyId) const;

	// Report the client callback all AABBs that are overlapping with
	// the given AABB. The client callback must return true if the query 
	// must be stopped or false to continue looking for more overlapping pairs.
	template<class T>
	void QueryAABB(T* callback, const b3AABB3& aabb) const;

	// Report the client callback all AABBs that are overlapping with
	// the given ray. The client callback must return the new intersection fraction 
	// (real). If the fraction == 0 then the query is cancelled immediatly.
	template<class T>
	void RayCast(T* callback, const b3RayCastInput& input) const;

	// Draw this tree.
	void Draw() const;

	u32 GetSize() const;
private :
	// A node in a static tree.
	struct b3Node
	{
		b3AABB3 aabb;
		u32 child1;
		union
		{
			u32 child2;
			u32 index;
		};

		// Is this node a leaf?
		bool IsLeaf() const
		{
			return child1 == B3_NULL_NODE_S;
		}
	};

	// 
	void Build(const b3AABB3* set, b3Node* node, u32* indices, u32 count, u32 minObjectsPerLeaf, u32 nodeCapacity, u32& leafCount, u32& internalCount);

	// The nodes of this tree stored in an array.
	u32 m_nodeCount;
	b3Node* m_nodes;
};

inline const b3AABB3& b3StaticTree::GetAABB(u32 proxyId) const
{
	B3_ASSERT(proxyId < m_nodeCount);
	return m_nodes[proxyId].aabb;
}

inline u32 b3StaticTree::GetUserData(u32 proxyId) const
{
	B3_ASSERT(proxyId < m_nodeCount);
	B3_ASSERT(m_nodes[proxyId].IsLeaf());
	return m_nodes[proxyId].index;
}

template<class T>
inline void b3StaticTree::QueryAABB(T* callback, const b3AABB3& aabb) const
{
	if (m_nodeCount == 0) 
	{
		return;
	}

	u32 root = 0;

	b3Stack<u32, 256> stack;
	stack.Push(root);

	while (stack.IsEmpty() == false) 
	{
		u32 nodeIndex = stack.Top();

		if (nodeIndex == B3_NULL_NODE_S)
		{
			continue;
		}

		stack.Pop();

		const b3Node* node = m_nodes + nodeIndex;

		if (b3TestOverlap(node->aabb, aabb) == true) 
		{
			if (node->IsLeaf() == true) 
			{
				if (callback->Report(nodeIndex) == false) 
				{
					return;
				}
			}
			else 
			{
				stack.Push(node->child1);
				stack.Push(node->child2);
			}
		}
	}
}

template<class T>
inline void b3StaticTree::RayCast(T* callback, const b3RayCastInput& input) const 
{
	if (m_nodeCount == 0)
	{
		return;
	}

	b3Vec3 p1 = input.p1;
	b3Vec3 p2 = input.p2;
	b3Vec3 d = p2 - p1;
	float32 maxFraction = input.maxFraction;

	// Ensure non-degenerate segment.
	B3_ASSERT(b3Dot(d, d) > B3_EPSILON * B3_EPSILON);

	u32 root = 0;

	b3Stack<u32, 256> stack;
	stack.Push(root);

	while (stack.IsEmpty() == false) 
	{
		u32 nodeIndex = stack.Top();	
		stack.Pop();

		if (nodeIndex == B3_NULL_NODE_S)
		{
			continue;
		}

		const b3Node* node = m_nodes + nodeIndex;

		float32 minFraction;
		if (node->aabb.TestRay(minFraction, p1, p2, maxFraction) == true)
		{
			if (node->IsLeaf() == true) 
			{
				b3RayCastInput subInput;
				subInput.p1 = input.p1;
				subInput.p2 = input.p2;
				subInput.maxFraction = maxFraction;

				float32 newFraction = callback->Report(subInput, nodeIndex);

				if (newFraction == 0.0f) 
				{
					// The client has stopped the query.
					return;
				}
			}
			else 
			{
				stack.Push(node->child1);
				stack.Push(node->child2);
			}
		}
	}
}

inline u32 b3StaticTree::GetSize() const
{
	u32 size = 0;
	size += sizeof(b3StaticTree);
	size += m_nodeCount * sizeof(b3Node);
	return size;
}

#endif
