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

#include <bounce/collision/shapes/mesh.h>

b3Mesh::b3Mesh()
{
	triangleWings = nullptr;
}

b3Mesh::~b3Mesh()
{
	b3Free(triangleWings);
}

void b3Mesh::BuildTree()
{
	b3AABB* aabbs = (b3AABB*)b3Alloc(triangleCount * sizeof(b3AABB));
	for (u32 i = 0; i < triangleCount; ++i)
	{
		aabbs[i] = GetTriangleAABB(i);
	}

	tree.Build(aabbs, triangleCount);

	b3Free(aabbs);
}

void b3Mesh::BuildAdjacency()
{
	B3_ASSERT(triangleWings == nullptr);
	triangleWings = (b3MeshTriangleWings*)b3Alloc(triangleCount * sizeof(b3MeshTriangleWings));
	
	// Assume the edges are open edges.
	for (u32 i = 0; i < triangleCount; ++i)
	{
		b3MeshTriangleWings* ws = triangleWings + i;
		
		ws->u1 = B3_NULL_VERTEX;
		ws->u2 = B3_NULL_VERTEX;
		ws->u3 = B3_NULL_VERTEX;
	}

	// Connect the edges.
	for (u32 i = 0; i < triangleCount; ++i)
	{
		b3MeshTriangle* t1 = triangles + i;
		b3MeshTriangleWings* t1ws = triangleWings + i;

		for (u32 j1 = 0; j1 < 3; ++j1)
		{
			u32 k1 = j1 + 1 < 3 ? j1 + 1 : 0;

			u32 t1v1 = t1->GetVertex(j1);
			u32 t1v2 = t1->GetVertex(k1);

			u32& u1 = t1ws->GetVertex(j1);

			if (u1 != B3_NULL_VERTEX)
			{
				// The edge is already connected.
				continue;
			}

			for (u32 j = i + 1; j < triangleCount; ++j)
			{
				b3MeshTriangle* t2 = triangles + j;
				b3MeshTriangleWings* t2ws = triangleWings + j;

				for (u32 j2 = 0; j2 < 3; ++j2)
				{
					u32 k2 = j2 + 1 < 3 ? j2 + 1 : 0;

					u32 t2v1 = t2->GetVertex(j2);
					u32 t2v2 = t2->GetVertex(k2);

					u32& u2 = t2ws->GetVertex(j2);

					if (t1v1 == t2v2 && t1v2 == t2v1)
					{
						// Both triangles have the same order.

						// The triangles are adjacent.
						
						// Non-shared vertex on triangle 1. 
						u32 k3 = k1 + 1 < 3 ? k1 + 1 : 0;
						u2 = t1->GetVertex(k3);

						// Non-shared vertex on triangle 2. 
						u32 k4 = k2 + 1 < 3 ? k2 + 1 : 0;
						u1 = t2->GetVertex(k4);

						break;
					}

					if (t1v1 == t2v1 && t1v2 == t2v2)
					{
						// The triangles have different order.

						// The triangles are adjacent.

						// Non-shared vertex on triangle 1. 
						u32 k3 = k1 + 1 < 3 ? k1 + 1 : 0;
						u2 = t1->GetVertex(k3);

						// Non-shared vertex on triangle 2. 
						u32 k4 = k2 + 1 < 3 ? k2 + 1 : 0;
						u1 = t2->GetVertex(k4);

						break;
					}
				}

				if (u1 != B3_NULL_VERTEX)
				{
					// The edge has been connected.
					break;
				}
			}
		}
	}
}

void b3Mesh::Scale(const b3Vec3& scale)
{
	for (u32 i = 0; i < vertexCount; ++i)
	{
		vertices[i] = b3MulCW(scale, vertices[i]);
	}
}

void b3Mesh::Rotate(const b3Quat& rotation)
{
	for (u32 i = 0; i < vertexCount; ++i)
	{
		vertices[i] = b3Mul(rotation, vertices[i]);
	}
}

void b3Mesh::Translate(const b3Vec3& translation)
{
	for (u32 i = 0; i < vertexCount; ++i)
	{
		vertices[i] += translation;
	}
}

void b3Mesh::Transform(const b3Transform& xf, const b3Vec3& scale)
{
	for (u32 i = 0; i < vertexCount; ++i)
	{
		vertices[i] = b3Mul(xf, b3MulCW(scale, vertices[i]));
	}
}