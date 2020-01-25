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

#include <bounce/softbody/softbody_mesh.h>
#include <bounce/meshgen/sphere_mesh.h>
#include <bounce/meshgen/cylinder_mesh.h>

b3QSoftBodyMesh::b3QSoftBodyMesh()
{
	vertexCount = 0;
	vertices = nullptr;
	triangleCount = 0;
	triangles = nullptr;
	tetrahedronCount = 0;
	tetrahedrons = nullptr;
}

b3QSoftBodyMesh::~b3QSoftBodyMesh()
{
	b3Free(vertices);
	b3Free(triangles);
	b3Free(tetrahedrons);
}

void b3QSoftBodyMesh::SetAsSphere(scalar radius, u32 subdivisions)
{
	smMesh mesh;
	smCreateMesh(mesh, subdivisions);

	B3_ASSERT(vertexCount == 0);
	vertexCount = 1 + mesh.vertexCount;
	vertices = (b3Vec3*)b3Alloc(vertexCount * sizeof(b3Vec3));
	vertices[0].SetZero();
	for (u32 i = 0; i < mesh.vertexCount; ++i)
	{
		vertices[1 + i] = mesh.vertices[i];
	}

	B3_ASSERT(triangleCount == 0);
	triangleCount = mesh.indexCount / 3;
	triangles = (b3SoftBodyMeshTriangle*)b3Alloc(triangleCount * sizeof(b3SoftBodyMeshTriangle));
	for (u32 i = 0; i < mesh.indexCount / 3; ++i)
	{
		u32 v1 = mesh.indices[3 * i + 0];
		u32 v2 = mesh.indices[3 * i + 1];
		u32 v3 = mesh.indices[3 * i + 2];

		b3SoftBodyMeshTriangle* t = triangles + i;

		t->v1 = 1 + v1;
		t->v2 = 1 + v2;
		t->v3 = 1 + v3;
	}

	B3_ASSERT(tetrahedronCount == 0);
	tetrahedronCount = mesh.indexCount / 3;
	tetrahedrons = (b3SoftBodyMeshTetrahedron*)b3Alloc(tetrahedronCount * sizeof(b3SoftBodyMeshTetrahedron));
	for (u32 i = 0; i < mesh.indexCount / 3; ++i)
	{
		u32 v1 = mesh.indices[3 * i + 0];
		u32 v2 = mesh.indices[3 * i + 1];
		u32 v3 = mesh.indices[3 * i + 2];

		b3SoftBodyMeshTetrahedron* t = tetrahedrons + i;

		t->v1 = 1 + v1;
		t->v2 = 1 + v2;
		t->v3 = 1 + v3;
		t->v4 = 0;
	}

	for (u32 i = 0; i < vertexCount; ++i)
	{
		vertices[i] *= radius;
	}
}

void b3QSoftBodyMesh::SetAsCylinder(scalar radius, scalar ey, u32 segments)
{
	cymMesh mesh;
	cymCreateMesh(mesh, segments);

	B3_ASSERT(vertexCount == 0);
	vertexCount = 1 + mesh.vertexCount;
	vertices = (b3Vec3*)b3Alloc(vertexCount * sizeof(b3Vec3));
	vertices[0].SetZero();
	for (u32 i = 0; i < mesh.vertexCount; ++i)
	{
		vertices[1 + i] = mesh.vertices[i];
	}

	B3_ASSERT(triangleCount == 0);
	triangleCount = mesh.indexCount / 3;
	triangles = (b3SoftBodyMeshTriangle*)b3Alloc(triangleCount * sizeof(b3SoftBodyMeshTriangle));
	for (u32 i = 0; i < mesh.indexCount / 3; ++i)
	{
		u32 v1 = mesh.indices[3 * i + 0];
		u32 v2 = mesh.indices[3 * i + 1];
		u32 v3 = mesh.indices[3 * i + 2];

		b3SoftBodyMeshTriangle* t = triangles + i;

		t->v1 = 1 + v1;
		t->v2 = 1 + v2;
		t->v3 = 1 + v3;
	}
	
	B3_ASSERT(tetrahedronCount == 0);
	tetrahedronCount = mesh.indexCount / 3;
	tetrahedrons = (b3SoftBodyMeshTetrahedron*)b3Alloc(tetrahedronCount * sizeof(b3SoftBodyMeshTetrahedron));
	for (u32 i = 0; i < mesh.indexCount / 3; ++i)
	{
		u32 v1 = mesh.indices[3 * i + 0];
		u32 v2 = mesh.indices[3 * i + 1];
		u32 v3 = mesh.indices[3 * i + 2];

		b3SoftBodyMeshTetrahedron* t = tetrahedrons + i;

		t->v1 = 1 + v1;
		t->v2 = 1 + v2;
		t->v3 = 1 + v3;
		t->v4 = 0;
	}

	scalar height = scalar(2) * ey;

	for (u32 i = 0; i < vertexCount; ++i)
	{
		vertices[i].x *= radius;
		vertices[i].y *= height;
		vertices[i].z *= radius;
	}
}