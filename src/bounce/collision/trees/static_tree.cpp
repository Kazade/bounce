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

#include <bounce/collision/trees/static_tree.h>
#include <bounce/common/template/stack.h>
#include <bounce/common/draw.h>

b3StaticTree::b3StaticTree()
{
	m_nodes = NULL;
	m_nodeCount = 0;
}

b3StaticTree::~b3StaticTree()
{
	b3Free(m_nodes);
}

static B3_FORCE_INLINE bool b3SortPredicate(const b3AABB3* set, u32 axis, u32 a, u32 b)
{
	b3Vec3 c1 = set[a].Centroid();
	b3Vec3 c2 = set[b].Centroid();

	return c1[axis] < c2[axis];
}

static void b3Sort(const b3AABB3* set, u32 axis, u32* ids, u32 count)
{
	if (count <= 1)
	{
		return;
	}

	u32 pivot = ids[count - 1];
	u32 low = 0;
	for (u32 i = 0; i < count - 1; ++i)
	{
		if (b3SortPredicate(set, axis, ids[i], pivot))
		{
			u32 tmp = ids[i];
			ids[i] = ids[low];
			ids[low] = tmp;
			low++;
		}
	}

	ids[count - 1] = ids[low];
	ids[low] = pivot;
	
	b3Sort(set, axis, ids, low);
	b3Sort(set, axis, ids + low + 1, count - 1 - low);
}

static u32 b3Partition(const b3AABB3& setAABB, const b3AABB3* set, u32* ids, u32 count)
{
	// Choose a partitioning axis.
	u32 splitAxis = setAABB.GetLongestAxisIndex();

	// Choose a split point.
	float32 splitPos = setAABB.Centroid()[splitAxis];

	// Sort along the split axis.
	b3Sort(set, splitAxis, ids, count);

	// Find the AABB that splits the set in two subsets.
	u32 left = 0;
	u32 right = count - 1;
	u32 middle = left;
	while (middle < right)
	{
		b3Vec3 center = set[ids[middle]].Centroid();
		if (center[splitAxis] > splitPos)
		{
			// Found median.
			break;
		}
		++middle;
	}

	// Ensure nonempty subsets.
	u32 count1 = middle;
	u32 count2 = count - middle;
	if (count1 == 0 || count2 == 0)
	{
		// Choose median.
		middle = (left + right) / 2;
	}

	return middle;
}

void b3StaticTree::Build(const b3AABB3* set, b3Node* node, u32* ids, u32 count, u32 minObjectsPerLeaf, u32 nodeCapacity, u32& leafCount, u32& internalCount)
{
	B3_ASSERT(count > 0);
	
	// Enclose set
	b3AABB3 setAABB = set[ids[0]];
	for (u32 i = 1; i < count; ++i)
	{
		setAABB = b3Combine(setAABB, set[ids[i]]);
	}

	node->aabb = setAABB;

	if (count <= minObjectsPerLeaf)
	{
		++leafCount;
		node->child1 = B3_NULL_NODE_S;
		node->index = ids[0];
	}
	else
	{
		++internalCount;

		// Partition current set
		u32 middle = b3Partition(setAABB, set, ids, count);

		// Allocate left subtree
		B3_ASSERT(m_nodeCount < nodeCapacity);
		node->child1 = m_nodeCount;
		++m_nodeCount;

		// Allocate right subtree
		B3_ASSERT(m_nodeCount < nodeCapacity);
		node->child2 = m_nodeCount;
		++m_nodeCount;

		// Build left and right subtrees
		Build(set, m_nodes + node->child1, ids, middle, minObjectsPerLeaf, nodeCapacity, leafCount, internalCount);
		Build(set, m_nodes + node->child2, ids + middle, count - middle, minObjectsPerLeaf, nodeCapacity, leafCount, internalCount);
	}
}

void b3StaticTree::Build(const b3AABB3* set, u32 count)
{
	B3_ASSERT(count > 0);

	u32* ids = (u32*)b3Alloc(count * sizeof(u32));
	for (u32 i = 0; i < count; ++i)
	{
		ids[i] = i;
	}

	// Leafs = n, Internals = n - 1, Total = 2n - 1, if we assume
	// each leaf node contains exactly 1 object.
	const u32 kMinObjectsPerLeaf = 1;

	u32 internalCapacity = count - 1;
	u32 leafCapacity = count;
	u32 nodeCapacity = 2 * count - 1;

	u32 internalCount = 0;
	u32 leafCount = 0;

	m_nodes = (b3Node*)b3Alloc(nodeCapacity * sizeof(b3Node));
	m_nodeCount = 1;

	Build(set, m_nodes, ids, count, kMinObjectsPerLeaf, nodeCapacity, leafCount, internalCount);

	b3Free(ids);

	B3_ASSERT(leafCount == leafCapacity);
	B3_ASSERT(internalCount == internalCapacity);
	B3_ASSERT(m_nodeCount == nodeCapacity);
}

void b3StaticTree::Draw() const
{
	if (m_nodeCount == 0)
	{
		return;
	}

	u32 root = 0;

	b3Stack<u32, 256> stack;
	stack.Push(root);

	while (!stack.IsEmpty())
	{
		u32 nodeIndex = stack.Top();

		stack.Pop();

		const b3Node* node = m_nodes + nodeIndex;
		if (node->IsLeaf())
		{
			b3Draw_draw->DrawAABB(node->aabb, b3Color_pink);
		}
		else
		{
			b3Draw_draw->DrawAABB(node->aabb, b3Color_red);
			
			stack.Push(node->child1);
			stack.Push(node->child2);
		}
	}
}
