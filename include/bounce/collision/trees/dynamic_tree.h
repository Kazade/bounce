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

#ifndef B3_DYNAMIC_TREE_H
#define B3_DYNAMIC_TREE_H

#include <bounce/common/draw.h>
#include <bounce/common/template/stack.h>
#include <bounce/collision/shapes/aabb3.h>
#include <bounce/collision/distance.h>

#define NULL_NODE (-1)

// AABB tree for dynamic AABBs.
class b3DynamicTree 
{
public :
	b3DynamicTree();
	~b3DynamicTree();

	// Insert a node into the tree and return its ID.
	i32 InsertNode(const b3AABB3& aabb, void* userData);
	
	// Remove a node from the tree.
	void RemoveNode(i32 proxyId);

	// Update a node AABB.
	void UpdateNode(i32 proxyId, const b3AABB3& aabb);

	// Get the (fat) AABB of a given proxy.
	const b3AABB3& GetAABB(i32 proxyId) const;

	// Get the data associated with a given proxy.
	void* GetUserData(i32 proxyId) const;

	// Check if two aabbs in this tree are overlapping.
	bool TestOverlap(i32 proxy1, i32 proxy2) const;

	// Keep reporting the client callback the AABBs that are overlapping with
	// the given AABB. The client callback must return true if the query 
	// must be stopped or false to continue looking for more overlapping pairs.
	template<class T> 
	void QueryAABB(T* callback, const b3AABB3& aabb) const;

	// Keep reporting the client callback all AABBs that are overlapping with
	// the given ray. The client callback must return the new intersection fraction.
	// If the fraction == 0 then the query is cancelled immediately.
	template<class T>
	void RayCast(T* callback, const b3RayCastInput& input) const;

	// Validate a given node of this tree.
	void Validate(i32 node) const;

	// Draw this tree.
	void Draw(b3Draw* draw) const;
private :
	struct b3Node 
	{
		// Is this node a leaf?
		bool IsLeaf() const 
		{
			//A node is a leaf if child 2 == NULL_NODE or height == 0.
			return child1 == NULL_NODE;
		}

		// The fattened node AABB.
		b3AABB3 aabb;

		// The associated user data.
		void* userData;

		union 
		{
			i32 parent;
			i32 next;
		};

		i32 child1;
		i32 child2;

		// leaf if 0, free node if -1
		i32 height;
	};
	
	// Insert a node into the tree.
	void InsertLeaf(i32 node);
	
	// Remove a node from the tree.
	void RemoveLeaf(i32 node);

	// Rebuild the hierarchy starting from the given node.
	void WalkBackNodeAndCombineVolumes(i32 node);
	
	// Find the best node that can be merged with a given AABB.
	i32 FindBest(const b3AABB3& aabb) const;

	// Peel a node from the free list and insert into the node array. 
	// Allocate a new node if necessary. The function returns the new node index.
	i32 AllocateNode();

	// Free a node from the node pool and add it to the free list.
	void FreeNode(i32 node);

	// Make a node available for the next allocation.
	void AddToFreeList(i32 node);

	// The root of this tree.
	i32 m_root;

	// The nodes of this tree stored in an array.
	b3Node* m_nodes;
	i32 m_nodeCount;
	i32 m_nodeCapacity;
	i32 m_freeList;
};

inline const b3AABB3& b3DynamicTree::GetAABB(i32 proxyId) const
{
	B3_ASSERT(proxyId < m_nodeCount);
	return m_nodes[proxyId].aabb;
}

inline void* b3DynamicTree::GetUserData(i32 proxyId) const
{
	B3_ASSERT(proxyId < m_nodeCount);
	return m_nodes[proxyId].userData;
}

inline bool b3DynamicTree::TestOverlap(i32 proxy1, i32 proxy2) const 
{
	B3_ASSERT(proxy1 < m_nodeCount);
	B3_ASSERT(proxy2 < m_nodeCount);
	return b3TestOverlap(m_nodes[proxy1].aabb, m_nodes[proxy2].aabb);
}

template<class T>
inline void b3DynamicTree::QueryAABB(T* callback, const b3AABB3& aabb) const 
{
	b3Stack<i32, 256> stack;
	stack.Push(m_root);

	while (stack.IsEmpty() == false) 
	{
		i32 nodeIndex = stack.Top();
		stack.Pop();

		if (nodeIndex == NULL_NODE) 
		{
			continue;
		}

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
inline void b3DynamicTree::RayCast(T* callback, const b3RayCastInput& input) const 
{
	b3Vec3 p1 = input.p1;
	b3Vec3 p2 = input.p2;
	b3Vec3 d = p2 - p1;
	float32 maxFraction = input.maxFraction;

	// Ensure non-degenerate segment.
	B3_ASSERT(b3Dot(d, d) > B3_EPSILON * B3_EPSILON);

	b3Stack<i32, 256> stack;
	stack.Push(m_root);

	while (stack.IsEmpty() == false) 
	{
		i32 nodeIndex = stack.Top();
		
		stack.Pop();

		if (nodeIndex == NULL_NODE) 
		{
			continue;
		}

		const b3Node* node = m_nodes + nodeIndex;

		float32 minFraction = 0.0f;
		if (node->aabb.TestRay(p1, p2, maxFraction, minFraction) == true) 
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

#endif