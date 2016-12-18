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

#include <bounce\collision\trees\static_tree.h>
#include <bounce\common\template\stack.h>
#include <algorithm>

b3StaticTree::b3StaticTree()
{
	m_nodes = nullptr;
	m_nodeCount = 0;
}

b3StaticTree::~b3StaticTree()
{
	b3Free(m_nodes);
}

void b3StaticTree::Build(u32* ids, const b3AABB3* set, u32 N)
{
	B3_ASSERT(N > 0);

	// Leafs = N, Internals = N - 1, Total = 2N - 1, if we assume
	// each leaf node contains exactly 1 object.
	const u32 kMinObjectsPerLeaf = 1;

	u32 internalCapacity = N - 1;
	u32 leafCapacity = N;
	u32 nodeCapacity = 2 * N - 1;

	u32 internalCount = 0;
	u32 leafCount = 0;

	m_nodes = (b3Node*)b3Alloc(nodeCapacity * sizeof(b3Node));
	m_nodeCount = 1;

	struct b3Params
	{
		u32 node;
		u32* indices;
		u32 numObjects;
	};

	struct b3SortPredicate
	{
		const b3AABB3* bs;
		u32 axis;

		bool operator()(const u32& i, const u32& j) const
		{
			const b3AABB3* b1 = bs + i;
			const b3AABB3* b2 = bs + j;

			b3Vec3 c1 = b1->Centroid();
			b3Vec3 c2 = b2->Centroid();

			if (c1[axis] < c2[axis])
			{
				return true;
			}

			return false;
		}
	};

	b3Stack<b3Params, 256> stack;

	{
		b3Params params;
		params.node = 0;
		params.indices = ids;
		params.numObjects = N;
		stack.Push(params);
	}

	while (stack.Count() > 0)
	{
		b3Params params = stack.Top();
		stack.Pop();

		u32 nodeIndex = params.node;
		u32* indices = params.indices;
		u32 numObjects = params.numObjects;

		B3_ASSERT(numObjects > 0);
		
		// "Allocate" node
		b3Node* node = m_nodes + nodeIndex;

		// Enclose set
		b3AABB3 setAABB = set[indices[0]];
		for (u32 i = 1; i < numObjects; ++i)
		{
			setAABB = b3Combine(setAABB, set[indices[i]]);
		}

		node->aabb = setAABB;

		if (numObjects <= kMinObjectsPerLeaf)
		{
			++leafCount;
			node->child1 = NULL_NODE_S;
			node->index = indices[0];
		}
		else
		{
			++internalCount;

			u32 splitAxis = setAABB.GetLongestAxisIndex();
			float32 splitPos = setAABB.Centroid()[splitAxis];

			// Sort along longest axis
			b3SortPredicate pred;
			pred.axis = splitAxis;
			pred.bs = set;
			std::sort(indices, indices + numObjects, pred);

			// Find the object that splits the set in two subsets.
			u32 left = 0;
			u32 right = numObjects - 1;
			u32 middle = left;
			while (middle < right)
			{
				b3Vec3 center = set[indices[middle]].Centroid();
				if (center[splitAxis] > splitPos)
				{
					// Found median.
					break;
				}
				++middle;
			}
			
			B3_ASSERT(middle >= left);
			B3_ASSERT(middle <= right);

			// Ensure we don't have empty subsets.
			u32 count1 = middle;
			u32 count2 = numObjects - middle;
			if (count1 == 0 || count2 == 0)
			{
				// Split at object median.
				middle = (left + right) / 2;
				count1 = middle;
				count2 = numObjects - middle;
			}

			B3_ASSERT(count1 > 0 && count2 > 0);

			B3_ASSERT(m_nodeCount < nodeCapacity);
			node->child1 = m_nodeCount;
			++m_nodeCount;

			B3_ASSERT(m_nodeCount < nodeCapacity);
			node->child2 = m_nodeCount;
			++m_nodeCount;

			// Repeat for childs
			b3Params params1;
			params1.node = node->child1;
			params1.indices = indices;
			params1.numObjects = count1;
			stack.Push(params1);

			b3Params params2;
			params2.node = node->child2;
			params2.indices = indices + middle;
			params2.numObjects = count2;
			stack.Push(params2);
		}
	}
	
	B3_ASSERT(leafCount == leafCapacity);
	B3_ASSERT(internalCount == internalCapacity);
	B3_ASSERT(m_nodeCount == nodeCapacity);
}

void b3StaticTree::Draw(b3Draw* b3Draw) const
{
	b3Color red = b3Color(1.0f, 0.0f, 0.0f, 1.0f);
	b3Color green = b3Color(0.0f, 1.0f, 0.0f, 1.0f);
	b3Color blue = b3Color(0.0f, 0.0f, 1.0f, 1.0f);
	b3Color purple = b3Color(1.0f, 0.0f, 1.0f, 1.0f);

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
			b3Draw->DrawAABB(node->aabb, purple);
		}
		else
		{
			b3Draw->DrawAABB(node->aabb, red);
			
			stack.Push(node->child1);
			stack.Push(node->child2);
		}
	}
}
