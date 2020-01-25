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

#include <bounce/collision/trees/static_tree.h>
#include <bounce/common/template/stack.h>
#include <bounce/common/draw.h>
#include <algorithm>

b3StaticTree::b3StaticTree()
{
	m_root = B3_NULL_NODE_S;
	m_nodes = nullptr;
	m_nodeCount = 0;
}

b3StaticTree::~b3StaticTree()
{
	b3Free(m_nodes);
}

struct b3SortPredicate
{
	b3SortPredicate() { }

	bool operator()(u32 a, u32 b)
	{
		b3Vec3 ca = set[a].GetCenter();
		b3Vec3 cb = set[b].GetCenter();

		return ca[axis] < cb[axis];
	}

	const b3AABB* set;
	u32 axis;
};

static u32 b3Partition(const b3AABB& setAABB, const b3AABB* set, u32* ids, u32 count)
{
	// Choose a partitioning axis.
	u32 splitAxis = setAABB.GetLongestAxisIndex();

	// Choose a split point.
	scalar splitPos = setAABB.GetCenter()[splitAxis];

	// Sort along the split axis.
	b3SortPredicate predicate;
	predicate.set = set;
	predicate.axis = splitAxis;

	std::sort(ids, ids + count, predicate);

	// Find the AABB that splits the set in two subsets.
	u32 left = 0;
	u32 right = count - 1;
	u32 middle = left;
	while (middle < right)
	{
		b3Vec3 center = set[ids[middle]].GetCenter();
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

void b3StaticTree::RecurseBuild(const b3AABB* set, b3Node* node, u32* ids, u32 count, u32 minObjectsPerLeaf, u32 nodeCapacity, u32& leafCount, u32& internalCount)
{
	B3_ASSERT(count > 0);
	
	// Enclose set
	b3AABB setAABB = set[ids[0]];
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
		RecurseBuild(set, m_nodes + node->child1, ids, middle, minObjectsPerLeaf, nodeCapacity, leafCount, internalCount);
		RecurseBuild(set, m_nodes + node->child2, ids + middle, count - middle, minObjectsPerLeaf, nodeCapacity, leafCount, internalCount);
	}
}

void b3StaticTree::Build(const b3AABB* set, u32 count)
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

	m_root = 0;
	m_nodes = (b3Node*)b3Alloc(nodeCapacity * sizeof(b3Node));
	m_nodeCount = 1;

	RecurseBuild(set, m_nodes, ids, count, kMinObjectsPerLeaf, nodeCapacity, leafCount, internalCount);

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

	b3Stack<u32, 256> stack;
	stack.Push(m_root);

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
