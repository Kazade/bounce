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

#ifndef B3_GRID_CLOTH_MESH_H
#define B3_GRID_CLOTH_MESH_H

#include <bounce/cloth/cloth_mesh.h>

// A (H + 1) x (W + 1) grid mesh stored in row-major order.
// v(i, j) = i + (W + 1) + j
template<u32 W = 1, u32 H = 1>
struct b3GridClothMesh : public b3ClothMesh
{
	b3Vec3 gridVertices[(H + 1) * (W + 1)];
	b3ClothMeshTriangle gridTriangles[2 * H * W];
	b3ClothMeshMesh gridMesh;

	// Set this grid to a W (width) per H (height) dimensioned grid centered at the origin and aligned
	// with the world x-z axes.
	b3GridClothMesh()
	{
		vertexCount = 0;
		for (u32 i = 0; i <= H; ++i)
		{
			for (u32 j = 0; j <= W; ++j)
			{
				gridVertices[vertexCount++].Set(float32(j), 0.0f, float32(i));
			}
		}

		B3_ASSERT(vertexCount == (W + 1) * (H + 1));
		
		b3Vec3 translation;
		translation.x = -0.5f * float32(W);
		translation.y = 0.0f;
		translation.z = -0.5f * float32(H);

		for (u32 i = 0; i < vertexCount; ++i)
		{
			gridVertices[i] += translation;
		}

		triangleCount = 0;
		for (u32 i = 0; i < H; ++i)
		{
			for (u32 j = 0; j < W; ++j)
			{
				u32 v1 = i * (W + 1) + j;
				u32 v2 = (i + 1) * (W + 1) + j;
				u32 v3 = (i + 1) * (W + 1) + (j + 1);
				u32 v4 = i * (W + 1) + (j + 1);

				b3ClothMeshTriangle* t1 = gridTriangles + triangleCount++;
				t1->v1 = v3;
				t1->v2 = v2;
				t1->v3 = v1;

				b3ClothMeshTriangle* t2 = gridTriangles + triangleCount++;
				t2->v1 = v1;
				t2->v2 = v4;
				t2->v3 = v3;
			}
		}
		
		B3_ASSERT(triangleCount == 2 * H * W);

		gridMesh.startTriangle = 0;
		gridMesh.triangleCount = triangleCount;
		gridMesh.startVertex = 0;
		gridMesh.vertexCount = vertexCount;

		vertices = gridVertices;
		triangles = gridTriangles;
		meshCount = 1;
		meshes = &gridMesh;
		sewingLineCount = 0;
		sewingLines = nullptr;
	}
};

#endif