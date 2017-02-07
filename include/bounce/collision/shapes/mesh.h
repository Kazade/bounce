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

#ifndef B3_MESH_H
#define B3_MESH_H

#include <bounce/collision/trees/static_tree.h>

struct b3Triangle
{
	// Test if this triangle contains a given vertex.
	bool TestVertex(u32 v) const
	{
		return v == v1 || v == v2 || v == v3;
	}

	// Test if this triangle contains two vertices.
	bool TestEdge(u32 _v1, u32 _v2) const
	{
		return TestVertex(_v1) && TestVertex(_v2);
	}

	u32 v1, v2, v3;
};

struct b3Mesh 
{
	u32 vertexCount;
	b3Vec3* vertices;
	u32 triangleCount;
	b3Triangle* triangles;
	b3StaticTree tree;

	const b3Vec3& GetVertex(u32 index) const;
	const b3Triangle& GetTriangle(u32 index) const;
	u32 GetSize() const;

	b3AABB3 GetTriangleAABB(u32 index) const;

	void BuildTree();
};

inline const b3Vec3& b3Mesh::GetVertex(u32 index) const
{
	return vertices[index];
}

inline const b3Triangle& b3Mesh::GetTriangle(u32 index) const
{
	return triangles[index];
}

inline u32 b3Mesh::GetSize() const
{
	u32 size = 0;
	size += sizeof(b3Mesh);
	size += sizeof(b3Vec3) * vertexCount;
	size += sizeof(b3Triangle) * triangleCount;
	size += tree.GetSize();
	return size;
}

inline b3AABB3 b3Mesh::GetTriangleAABB(u32 index) const
{
	const b3Triangle* triangle = triangles + index;
	
	u32 i1 = triangle->v1;
	u32 i2 = triangle->v2;
	u32 i3 = triangle->v3;

	b3AABB3 aabb;
	aabb.m_lower = b3Min(b3Min(vertices[i1], vertices[i2]), vertices[i3]);
	aabb.m_upper = b3Max(b3Max(vertices[i1], vertices[i2]), vertices[i3]);
	return aabb;
}

inline void b3Mesh::BuildTree()
{
	b3AABB3* aabbs = (b3AABB3*)b3Alloc(triangleCount * sizeof(b3AABB3));
	for (u32 i = 0; i < triangleCount; ++i)
	{
		aabbs[i] = GetTriangleAABB(i);
	}

	tree.Build(aabbs, triangleCount);

	b3Free(aabbs);
}

#endif
