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
template<u32 W = 1, u32 H = 1>
struct b3GridClothMesh : public b3ClothMesh
{
	b3Vec3 gridVertices[(H + 1) * (W + 1)];
	b3ClothMeshTriangle gridTriangles[2 * H * W];
	b3ClothMeshMesh gridMesh;
	b3ClothMeshShearingLine gridShearingLines[2 * H * W];
	b3ClothMeshBendingLine gridBendingLines[(H + 1) * (W - 1) + (W + 1) * (H - 1)];

	// Set this grid to a W (width) per H (height) dimensioned grid centered at the origin and aligned
	// with the world x-z axes.
	b3GridClothMesh()
	{
		vertexCount = 0;
		for (u32 j = 0; j <= W; ++j)
		{
			for (u32 i = 0; i <= H; ++i)
			{
				u32 vertex = GetVertex(i, j);
				gridVertices[vertex].Set(scalar(j), scalar(0), scalar(i));
				++vertexCount;
			}
		}

		B3_ASSERT(vertexCount == (H + 1) * (W + 1));

		b3Vec3 translation;
		translation.x = scalar(-0.5) * scalar(W);
		translation.y = scalar(0);
		translation.z = scalar(-0.5) * scalar(H);

		for (u32 i = 0; i < vertexCount; ++i)
		{
			gridVertices[i] += translation;
		}

		triangleCount = 0;
		for (u32 i = 0; i < H; ++i)
		{
			for (u32 j = 0; j < W; ++j)
			{
				// 1*|----|*4
				//   |----|
				// 2*|----|*3
				u32 v1 = GetVertex(i, j);
				u32 v2 = GetVertex(i + 1, j);
				u32 v3 = GetVertex(i + 1, j + 1);
				u32 v4 = GetVertex(i, j + 1);

				b3ClothMeshTriangle* t1 = gridTriangles + triangleCount++;
				t1->v1 = v1;
				t1->v2 = v2;
				t1->v3 = v3;

				b3ClothMeshTriangle* t2 = gridTriangles + triangleCount++;
				t2->v1 = v3;
				t2->v2 = v4;
				t2->v3 = v1;
			}
		}

		B3_ASSERT(triangleCount == 2 * H * W);

		shearingLineCount = 0;
		for (u32 i = 0; i < H; ++i)
		{
			for (u32 j = 0; j < W; ++j)
			{
				// 1*|----|*4
				//   |----|
				// 2*|----|*3
				u32 v1 = GetVertex(i, j);
				u32 v2 = GetVertex(i + 1, j);
				u32 v3 = GetVertex(i + 1, j + 1);
				u32 v4 = GetVertex(i, j + 1);

				gridShearingLines[shearingLineCount].v1 = v1;
				gridShearingLines[shearingLineCount].v2 = v3;
				++shearingLineCount;

				gridShearingLines[shearingLineCount].v1 = v4;
				gridShearingLines[shearingLineCount].v2 = v2;
				++shearingLineCount;
			}
		}

		B3_ASSERT(shearingLineCount == 2 * H * W);

		bendingLineCount = 0;
		for (u32 i = 0; i <= H; ++i)
		{
			for (u32 j = 0; j < W - 1; ++j)
			{
				u32 v1 = GetVertex(i, j);
				u32 v2 = GetVertex(i, j + 2);

				gridBendingLines[bendingLineCount].v1 = v1;
				gridBendingLines[bendingLineCount].v2 = v2;
				++bendingLineCount;
			}
		}

		for (u32 j = 0; j <= W; ++j)
		{
			for (u32 i = 0; i < H - 1; ++i)
			{
				u32 v1 = GetVertex(i, j);
				u32 v2 = GetVertex(i + 2, j);

				gridBendingLines[bendingLineCount].v1 = v1;
				gridBendingLines[bendingLineCount].v2 = v2;
				++bendingLineCount;
			}
		}

		B3_ASSERT(bendingLineCount == (H + 1) * (W - 1) + (W + 1) * (H - 1));

		gridMesh.startTriangle = 0;
		gridMesh.triangleCount = triangleCount;
		gridMesh.startVertex = 0;
		gridMesh.vertexCount = vertexCount;

		vertices = gridVertices;
		triangles = gridTriangles;
		meshCount = 1;
		meshes = &gridMesh;
		shearingLines = gridShearingLines;
		bendingLines = gridBendingLines;
		sewingLineCount = 0;
		sewingLines = nullptr;
	}

	u32 GetVertex(u32 i, u32 j)
	{
		B3_ASSERT(i < H + 1);
		B3_ASSERT(j < W + 1);
		return i * (W + 1) + j;
	}
};

#endif