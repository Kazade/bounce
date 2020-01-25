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

#ifndef B3_BLOCK_SOFT_BODY_MESH_H
#define B3_BLOCK_SOFT_BODY_MESH_H

#include <bounce/softbody/softbody_mesh.h>

template<u32 W = 1, u32 H = 1, u32 D = 1>
struct b3BlockSoftBodyMesh : public b3SoftBodyMesh
{
	b3Vec3 blockVertices[(H + 1) * (W + 1) * (D + 1)];
	b3SoftBodyMeshTriangle blockTriangles[4 * H * W + 4 * H * D + 4 * W * D];
	b3SoftBodyMeshTetrahedron blockTetrahedrons[5 * H * W * D];

	b3BlockSoftBodyMesh()
	{
		vertexCount = 0;

		for (u32 j = 0; j <= W; ++j)
		{
			for (u32 i = 0; i <= H; ++i)
			{
				for (u32 k = 0; k <= D; ++k)
				{
					u32 vertex = GetVertex(i, j, k);
					blockVertices[vertex].Set(scalar(j), scalar(i), scalar(k));
					++vertexCount;
				}
			}
		}

		B3_ASSERT(vertexCount == (H + 1) * (W + 1) * (D + 1));

		b3Vec3 translation;
		translation.x = scalar(-0.5) * scalar(W);
		translation.y = scalar(-0.5) * scalar(H);
		translation.z = scalar(-0.5) * scalar(D);

		for (u32 i = 0; i < vertexCount; ++i)
		{
			blockVertices[i] += translation;
		}

		triangleCount = 0;

		// x-y plane
		for (u32 i = 0; i < H; ++i)
		{
			for (u32 j = 0; j < W; ++j)
			{
				//     2*-----*3
				//     /|    /|
				//    / |   / |
				//   *-----*  |
				//   | 1*--|--*4
				//   | /   | /
				//   |/    |/
				//   *-----*
				{
					u32 v1 = GetVertex(i, j, 0);
					u32 v2 = GetVertex(i + 1, j, 0);
					u32 v3 = GetVertex(i + 1, j + 1, 0);
					u32 v4 = GetVertex(i, j + 1, 0);

					blockTriangles[triangleCount].v1 = v1;
					blockTriangles[triangleCount].v2 = v2;
					blockTriangles[triangleCount].v3 = v3;

					++triangleCount;

					blockTriangles[triangleCount].v1 = v3;
					blockTriangles[triangleCount].v2 = v4;
					blockTriangles[triangleCount].v3 = v1;

					++triangleCount;
				}

				{
					u32 v1 = GetVertex(i, j, D);
					u32 v2 = GetVertex(i + 1, j, D);
					u32 v3 = GetVertex(i + 1, j + 1, D);
					u32 v4 = GetVertex(i, j + 1, D);

					blockTriangles[triangleCount].v1 = v3;
					blockTriangles[triangleCount].v2 = v2;
					blockTriangles[triangleCount].v3 = v1;

					++triangleCount;

					blockTriangles[triangleCount].v1 = v1;
					blockTriangles[triangleCount].v2 = v4;
					blockTriangles[triangleCount].v3 = v3;

					++triangleCount;
				}
			}
		}

		// y-z plane
		for (u32 i = 0; i < H; ++i)
		{
			for (u32 k = 0; k < D; ++k)
			{
				//      4*-----* 
				//     /|     /|
				//    / |    / |
				//   3*-----*  |
				//   |  1*--|--* 
				//   | /   | /
				//   |/    |/
				//   2*-----*
				{
					u32 v1 = GetVertex(i, 0, k);
					u32 v2 = GetVertex(i, 0, k + 1);
					u32 v3 = GetVertex(i + 1, 0, k + 1);
					u32 v4 = GetVertex(i + 1, 0, k);

					blockTriangles[triangleCount].v1 = v1;
					blockTriangles[triangleCount].v2 = v2;
					blockTriangles[triangleCount].v3 = v3;

					++triangleCount;

					blockTriangles[triangleCount].v1 = v3;
					blockTriangles[triangleCount].v2 = v4;
					blockTriangles[triangleCount].v3 = v1;

					++triangleCount;
				}

				{
					u32 v1 = GetVertex(i, W, k);
					u32 v2 = GetVertex(i, W, k + 1);
					u32 v3 = GetVertex(i + 1, W, k + 1);
					u32 v4 = GetVertex(i + 1, W, k);

					blockTriangles[triangleCount].v1 = v3;
					blockTriangles[triangleCount].v2 = v2;
					blockTriangles[triangleCount].v3 = v1;

					++triangleCount;

					blockTriangles[triangleCount].v1 = v1;
					blockTriangles[triangleCount].v2 = v4;
					blockTriangles[triangleCount].v3 = v3;

					++triangleCount;
				}
			}
		}

		// x-z plane
		for (u32 j = 0; j < W; ++j)
		{
			for (u32 k = 0; k < D; ++k)
			{
				//      *-----* 
				//     /|     /|
				//    / |    / |
				//   *-----*   |
				//   |  1*----2* 
				//   | /   | /
				//   |/    |/
				//   4*-----3*
				{
					u32 v1 = GetVertex(0, j, k);
					u32 v2 = GetVertex(0, j + 1, k);
					u32 v3 = GetVertex(0, j + 1, k + 1);
					u32 v4 = GetVertex(0, j, k + 1);

					blockTriangles[triangleCount].v1 = v1;
					blockTriangles[triangleCount].v2 = v2;
					blockTriangles[triangleCount].v3 = v3;

					++triangleCount;

					blockTriangles[triangleCount].v1 = v3;
					blockTriangles[triangleCount].v2 = v4;
					blockTriangles[triangleCount].v3 = v1;

					++triangleCount;
				}

				{
					u32 v1 = GetVertex(H, j, k);
					u32 v2 = GetVertex(H, j + 1, k);
					u32 v3 = GetVertex(H, j + 1, k + 1);
					u32 v4 = GetVertex(H, j, k + 1);

					blockTriangles[triangleCount].v1 = v3;
					blockTriangles[triangleCount].v2 = v2;
					blockTriangles[triangleCount].v3 = v1;

					++triangleCount;

					blockTriangles[triangleCount].v1 = v1;
					blockTriangles[triangleCount].v2 = v4;
					blockTriangles[triangleCount].v3 = v3;

					++triangleCount;
				}
			}
		}

		B3_ASSERT(triangleCount == 4 * H * W + 4 * H * D + 4 * W * D);

		tetrahedronCount = 0;
		for (u32 i = 0; i < H; ++i)
		{
			for (u32 j = 0; j < W; ++j)
			{
				for (u32 k = 0; k < D; ++k)
				{
					//     4*-----8* 
					//     /|     /|
					//    / |    / |
					//  3*-----7*  |
					//   | 1*--|--5*
					//   | /   |  /
					//   |/    | /
					//  2*-----6*

					u32 v1 = GetVertex(i, j, k);
					u32 v2 = GetVertex(i, j, k + 1);
					u32 v3 = GetVertex(i + 1, j, k + 1);
					u32 v4 = GetVertex(i + 1, j, k);

					u32 v5 = GetVertex(i, j + 1, k);
					u32 v6 = GetVertex(i, j + 1, k + 1);
					u32 v7 = GetVertex(i + 1, j + 1, k + 1);
					u32 v8 = GetVertex(i + 1, j + 1, k);

					if ((i + j + k) % 2 == 1)
					{
						blockTetrahedrons[tetrahedronCount++] = { v2, v6, v7, v5 };
						blockTetrahedrons[tetrahedronCount++] = { v5, v7, v4, v8 };
						blockTetrahedrons[tetrahedronCount++] = { v2, v4, v7, v3 };
						blockTetrahedrons[tetrahedronCount++] = { v2, v5, v4, v1 };
						blockTetrahedrons[tetrahedronCount++] = { v2, v7, v4, v5 };
					}
					else
					{
						blockTetrahedrons[tetrahedronCount++] = { v6, v1, v3, v2 };
						blockTetrahedrons[tetrahedronCount++] = { v6, v8, v1, v5 };
						blockTetrahedrons[tetrahedronCount++] = { v6, v3, v8, v7 };
						blockTetrahedrons[tetrahedronCount++] = { v1, v8, v3, v4 };
						blockTetrahedrons[tetrahedronCount++] = { v6, v1, v8, v3 };
					}
				}
			}
		}

		B3_ASSERT(tetrahedronCount == 5 * H * W * D);

		vertices = blockVertices;
		triangles = blockTriangles;
		tetrahedrons = blockTetrahedrons;
	}

	u32 GetVertex(u32 i, u32 j, u32 k)
	{
		B3_ASSERT(i < H + 1);
		B3_ASSERT(j < W + 1);
		B3_ASSERT(k < D + 1);
		return k + (D + 1) * (j + (W + 1) * i);
	}
};

#endif