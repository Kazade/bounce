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

#ifndef SPHERE_MESH_H
#define SPHERE_MESH_H

#include <bounce/bounce.h>

//
struct smMesh
{
	smMesh() { }

	smMesh& operator=(const smMesh& other)
	{
		Copy(other);
		return *this;
	}

	void Copy(const smMesh& other)
	{
		vertices = other.vertices;
		triangleIndices = other.triangleIndices;
	}

	void AddVertex(float32 x, float32 y, float32 z)
	{
		vertices.PushBack(b3Vec3(x, y, z));
	}

	void AddTriangle(u32 v1, u32 v2, u32 v3)
	{
		triangleIndices.PushBack(v1);
		triangleIndices.PushBack(v2);
		triangleIndices.PushBack(v3);
	}

	void SetAsIcosahedron()
	{
		assert(vertices.Count() == 0);

		float32 t = 0.5f * (1.0f + b3Sqrt(5.0f));

		AddVertex(-1.0f, t, 0.0f);
		AddVertex(1.0f, t, 0.0f);
		AddVertex(-1.0f, -t, 0.0f);
		AddVertex(1.0f, -t, 0.0f);

		AddVertex(0.0f, -1.0f, t);
		AddVertex(0.0f, 1.0f, t);
		AddVertex(0.0f, -1.0f, -t);
		AddVertex(0.0f, 1.0f, -t);

		AddVertex(t, 0.0f, -1.0f);
		AddVertex(t, 0.0f, 1.0f);
		AddVertex(-t, 0.0f, -1.0f);
		AddVertex(-t, 0.0f, 1.0f);

		for (u32 i = 0; i < vertices.Count(); ++i)
		{
			vertices[i].Normalize();
		}

		assert(triangleIndices.Count() == 0);

		AddTriangle(0, 11, 5);
		AddTriangle(0, 5, 1);
		AddTriangle(0, 1, 7);
		AddTriangle(0, 7, 10);
		AddTriangle(0, 10, 11);

		AddTriangle(1, 5, 9);
		AddTriangle(5, 11, 4);
		AddTriangle(11, 10, 2);
		AddTriangle(10, 7, 6);
		AddTriangle(7, 1, 8);

		AddTriangle(3, 9, 4);
		AddTriangle(3, 4, 2);
		AddTriangle(3, 2, 6);
		AddTriangle(3, 6, 8);
		AddTriangle(3, 8, 9);

		AddTriangle(4, 9, 5);
		AddTriangle(2, 4, 11);
		AddTriangle(6, 2, 10);
		AddTriangle(8, 6, 7);
		AddTriangle(9, 8, 1);
	}

	b3StackArray<b3Vec3, 256> vertices;
	b3StackArray<u32, 256> triangleIndices;
};

// Generate an icosphere given the number of subdivisions.
void smCreateMesh(smMesh& output, u32 subdivisions);

#endif SPHERE_MESH_H