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

#include <bounce/collision/trees/dynamic_tree.h>
#include <bounce/common/draw.h>

b3DynamicTree::b3DynamicTree() 
{
	m_root = B3_NULL_NODE_D;

	// Preallocate 32 nodes.
	m_nodeCapacity = 32;
	m_nodes = (b3Node*) b3Alloc(m_nodeCapacity * sizeof(b3Node));
	memset(m_nodes, 0, m_nodeCapacity * sizeof(b3Node));
	m_nodeCount = 0;

	// Link the allocated nodes and make the first node 
	// available the the next allocation.
	AddToFreeList(m_nodeCount);
}

b3DynamicTree::~b3DynamicTree() 
{
	b3Free(m_nodes);
}

// Return a node from the pool.
u32 b3DynamicTree::AllocateNode() 
{
	B3_ASSERT(m_nodeCapacity > 0);

	if (m_freeList == B3_NULL_NODE_D) 
	{
		B3_ASSERT(m_nodeCount == m_nodeCapacity);

		// Duplicate capacity.
		m_nodeCapacity *= 2;

		b3Node* oldNodes = m_nodes;
		m_nodes = (b3Node*) b3Alloc(m_nodeCapacity * sizeof(b3Node));;
		memcpy(m_nodes, oldNodes, m_nodeCount * sizeof(b3Node));
		b3Free(oldNodes);

		// Link the (allocated) nodes starting from the new 
		// node and make the new nodes available the the next allocation.
		AddToFreeList(m_nodeCount);
	}

	// Grab the free node.
	u32 node = m_freeList;

	m_freeList = m_nodes[node].next;

	m_nodes[node].parent = B3_NULL_NODE_D;
	m_nodes[node].child1 = B3_NULL_NODE_D;
	m_nodes[node].child2 = B3_NULL_NODE_D;
	m_nodes[node].height = 0;
	m_nodes[node].userData = NULL;

	++m_nodeCount;

	return node;
}

void b3DynamicTree::FreeNode(u32 node) 
{
	B3_ASSERT(node != B3_NULL_NODE_D && node < m_nodeCapacity);
	m_nodes[node].next = m_freeList;
	m_nodes[node].height = -1;
	m_freeList = node;
	--m_nodeCount;
}

void b3DynamicTree::AddToFreeList(u32 node) 
{
	B3_ASSERT(m_nodeCapacity > 0);
	
	// Starting from the given node, relink the linked list of nodes.
	for (u32 i = node; i < m_nodeCapacity - 1; ++i) 
	{
		m_nodes[i].next = i + 1;
		m_nodes[i].height = -1;
	}

	m_nodes[m_nodeCapacity - 1].next = B3_NULL_NODE_D;
	m_nodes[m_nodeCapacity - 1].height = -1;

	// Make the node available for the next allocation.
	m_freeList = node;
}

u32 b3DynamicTree::InsertNode(const b3AABB3& aabb, void* userData) 
{
	// Insert into the array.
	u32 node = AllocateNode();
	m_nodes[node].aabb = aabb;
	m_nodes[node].userData = userData;
	m_nodes[node].height = 0;

	// Insert into the tree.
	InsertLeaf(node);

	// Return the node ID.
	return node;
}

void b3DynamicTree::RemoveNode(u32 proxyId) 
{
	// Remove from the tree.
	RemoveLeaf(proxyId);
	// Remove from the node array and make it available.
	FreeNode(proxyId);
}

void b3DynamicTree::UpdateNode(u32 proxyId, const b3AABB3& aabb)
{
	B3_ASSERT(m_root != B3_NULL_NODE_D);
	B3_ASSERT(m_nodes[proxyId].IsLeaf());
	// Remove old AABB from the tree.
	RemoveLeaf(proxyId);
	// Insert the new AABB to the tree.
	m_nodes[proxyId].aabb = aabb;
	InsertLeaf(proxyId);
}

u32 b3DynamicTree::FindBest(const b3AABB3& leafAABB) const 
{
	u32 index = m_root;
	while (!m_nodes[index].IsLeaf()) 
	{
		float32 branchArea = m_nodes[index].aabb.SurfaceArea();

		// Minumum cost of pushing the leaf down the tree.
		b3AABB3 combinedAABB = b3Combine(leafAABB, m_nodes[index].aabb);
		float32 combinedArea = combinedAABB.SurfaceArea();

		// Cost for creating a new parent node.
		float32 branchCost = 2.0f * combinedArea;

		float32 inheritanceCost = 2.0f * (combinedArea - branchArea);

		// The branch node child nodes cost.
		u32 child1 = m_nodes[index].child1;
		u32 child2 = m_nodes[index].child2;

		// Cost of descending onto child1.
		float32 childCost1 = 0.0f;
		if (m_nodes[child1].IsLeaf()) 
		{
			b3AABB3 aabb = b3Combine(leafAABB, m_nodes[child1].aabb);
			childCost1 = aabb.SurfaceArea();
		}
		else 
		{
			b3AABB3 aabb = b3Combine(leafAABB, m_nodes[child1].aabb);
			float32 oldArea = m_nodes[child1].aabb.SurfaceArea();
			float32 newArea = aabb.SurfaceArea();
			childCost1 = (newArea - oldArea) + inheritanceCost;
		}

		// Cost of descending onto child1.
		float32 childCost2 = 0.0f;
		if (m_nodes[child2].IsLeaf()) 
		{
			b3AABB3 aabb = b3Combine(leafAABB, m_nodes[child2].aabb);
			childCost2 = aabb.SurfaceArea();
		}
		else 
		{
			b3AABB3 aabb = b3Combine(leafAABB, m_nodes[child2].aabb);
			float32 oldArea = m_nodes[child2].aabb.SurfaceArea();
			float32 newArea = aabb.SurfaceArea();
			childCost2 = (newArea - oldArea) + inheritanceCost;
		}

		// Choose the node that has the minimum cost.
		if (branchCost < childCost1 && branchCost < childCost2) 
		{
			// The current branch node is the best node and it will be used.
			break;
		}

		// Visit the node that has the minimum cost.
		index = childCost1 < childCost2 ? child1 : child2;
	}
	return index;
}

void b3DynamicTree::InsertLeaf(u32 leaf) 
{
	if (m_root == B3_NULL_NODE_D) 
	{
		// If this tree root node is empty then just set the leaf
		// node to it.
		m_root = leaf;
		m_nodes[m_root].parent = B3_NULL_NODE_D;
		return;
	}

	// Get the inserted leaf AABB.
	b3AABB3 leafAabb = m_nodes[leaf].aabb;
	
	// Search for the best branch node of this tree starting from the tree root node.
	u32 sibling = FindBest(leafAabb);

	u32 oldParent = m_nodes[sibling].parent;
	
	// Create and setup new parent. 
	u32 newParent = AllocateNode();
	m_nodes[newParent].parent = oldParent;
	m_nodes[newParent].child1 = sibling;
	m_nodes[sibling].parent = newParent;	
	m_nodes[newParent].child2 = leaf;
	m_nodes[leaf].parent = newParent;	
	m_nodes[newParent].userData = NULL;
	m_nodes[newParent].aabb = b3Combine(leafAabb, m_nodes[sibling].aabb);
	m_nodes[newParent].height = m_nodes[sibling].height + 1;

	if (oldParent != B3_NULL_NODE_D) 
	{
		// The sibling was not the root.
		// Find which child node of the old parent is the sibling
		// and link the new parent to it.
		if (m_nodes[oldParent].child1 == sibling) 
		{
			m_nodes[oldParent].child1 = newParent;
		}
		else
		{
			m_nodes[oldParent].child2 = newParent;
		}
	}
	else 
	{
		// If the sibling was the root then the root becomes the created
		// node.
		m_root = newParent;
	}

	// If we have ancestor nodes then adjust its AABBs.
	WalkBackNodeAndCombineVolumes(newParent);
}

void b3DynamicTree::RemoveLeaf(u32 leaf) 
{
	if (leaf == m_root) 
	{
		m_root = B3_NULL_NODE_D;
		return;
	}

	u32 parent = m_nodes[leaf].parent;
	u32 grandParent = m_nodes[parent].parent;
	u32 sibling;
	if (m_nodes[parent].child1 == leaf) 
	{
		sibling = m_nodes[parent].child2;
	}
	else 
	{
		sibling = m_nodes[parent].child1;
	}

	if (grandParent != B3_NULL_NODE_D) 
	{
		if (m_nodes[grandParent].child1 == parent) 
		{
			m_nodes[grandParent].child1 = sibling;
		}
		else 
		{
			m_nodes[grandParent].child2 = sibling;
		}
		m_nodes[sibling].parent = grandParent;
		
		// Remove parent node.
		FreeNode(parent);

		// If we have ancestor then nodes adjust its AABBs.
		WalkBackNodeAndCombineVolumes(grandParent);
	}
	else 
	{
		m_root = sibling;
		m_nodes[sibling].parent = B3_NULL_NODE_D;		
		// Remove parent node.
		FreeNode(parent);
	}
}

void b3DynamicTree::WalkBackNodeAndCombineVolumes(u32 node) 
{
	while (node != B3_NULL_NODE_D) 
	{
		//@todo node = Balance(node);

		u32 child1 = m_nodes[node].child1;
		u32 child2 = m_nodes[node].child2;

		B3_ASSERT(child1 != B3_NULL_NODE_D);
		B3_ASSERT(child2 != B3_NULL_NODE_D);

		m_nodes[node].height = 1 + b3Max(m_nodes[child1].height, m_nodes[child2].height);
		m_nodes[node].aabb = b3Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);

		node = m_nodes[node].parent;
	}
}

void b3DynamicTree::Validate(u32 nodeID) const 
{
	if (nodeID == B3_NULL_NODE_D) 
	{
		return;
	}

	// The root node has no parent.
	if (nodeID == m_root) 
	{
		B3_ASSERT(m_nodes[nodeID].parent == B3_NULL_NODE_D);
	}

	const b3Node* node = m_nodes + nodeID;

	u32 child1 = node->child1;
	u32 child2 = node->child2;

	if (node->IsLeaf()) 
	{
		// Leaf nodes has no children and its height is zero.
		B3_ASSERT(child1 == B3_NULL_NODE_D);
		B3_ASSERT(child2 == B3_NULL_NODE_D);
		B3_ASSERT(node->height == 0);
	}
	else 
	{
		B3_ASSERT(0 <= child1 && child1 < m_nodeCapacity);
		B3_ASSERT(0 <= child2 && child2 < m_nodeCapacity);

		// The parent of its children is its parent (really?!).

		B3_ASSERT(m_nodes[child1].parent == nodeID);
		B3_ASSERT(m_nodes[child2].parent == nodeID);

		// Walk down the tree.
		Validate(child1);
		Validate(child2);
	}
}

void b3DynamicTree::Draw() const
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

		if (nodeIndex == B3_NULL_NODE_D)
		{
			continue;
		}

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
