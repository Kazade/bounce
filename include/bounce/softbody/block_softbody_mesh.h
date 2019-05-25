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
	b3Vec3 blockVertices[(W + 1) * (H + 1) * (D + 1)];
	b3SoftBodyMeshTetrahedron blockTetrahedrons[5 * W * H * D];

	b3BlockSoftBodyMesh()
	{
		vertexCount = 0;
		for (u32 x = 0; x <= W; ++x)
		{
			for (u32 y = 0; y <= H; ++y)
			{
				for (u32 z = 0; z <= D; ++z)
				{
					blockVertices[vertexCount++].Set(x, y, z);
				}
			}
		}

		B3_ASSERT(vertexCount == (W + 1) * (H + 1) * (D + 1));

		b3Vec3 translation;
		translation.x = -0.5f * float32(W);
		translation.y = -0.5f * float32(H);
		translation.z = -0.5f * float32(D);

		for (u32 i = 0; i < vertexCount; ++i)
		{
			blockVertices[i] += translation;
		}

		tetrahedronCount = 0;
		for (u32 x = 0; x < W; ++x)
		{
			for (u32 y = 0; y < H; ++y)
			{
				for (u32 z = 0; z < D; ++z)
				{
					//     4*-----*7
					//     /|    /|
					//    / |   / |
					//  5*-----*6 |
					//   | 0*--|--*3
					//   | /   | /
					//   |/    |/
					//  1*-----*2
					u32 v0 = (x * (H + 1) + y) * (D + 1) + z;
					u32 v1 = v0 + 1;
					u32 v3 = ((x + 1) * (H + 1) + y) * (D + 1) + z;
					u32 v2 = v3 + 1;
					u32 v7 = ((x + 1) * (H + 1) + (y + 1)) * (D + 1) + z;
					u32 v6 = v7 + 1;
					u32 v4 = (x * (H + 1) + (y + 1)) * (D + 1) + z;
					u32 v5 = v4 + 1;

					if ((x + y + z) % 2 == 1)
					{
						// CCW
						//blockTetrahedrons[tetrahedronCount++] = { v1, v2, v6, v3 };
						//blockTetrahedrons[tetrahedronCount++] = { v3, v6, v4, v7 };
						//blockTetrahedrons[tetrahedronCount++] = { v1, v4, v6, v5 };
						//blockTetrahedrons[tetrahedronCount++] = { v1, v3, v4, v0 };
						//blockTetrahedrons[tetrahedronCount++] = { v1, v6, v4, v3 };

						// CW
						blockTetrahedrons[tetrahedronCount++] = { v2, v1, v6, v3 };
						blockTetrahedrons[tetrahedronCount++] = { v6, v3, v4, v7 };
						blockTetrahedrons[tetrahedronCount++] = { v4, v1, v6, v5 };
						blockTetrahedrons[tetrahedronCount++] = { v3, v1, v4, v0 };
						blockTetrahedrons[tetrahedronCount++] = { v6, v1, v4, v3 };
					}
					else
					{
						// CCW
						//blockTetrahedrons[tetrahedronCount++] = { v2, v0, v5, v1 };
						//blockTetrahedrons[tetrahedronCount++] = { v2, v7, v0, v3 };
						//blockTetrahedrons[tetrahedronCount++] = { v2, v5, v7, v6 };
						//blockTetrahedrons[tetrahedronCount++] = { v0, v7, v5, v4 };
						//blockTetrahedrons[tetrahedronCount++] = { v2, v0, v7, v5 };

						// CW
						blockTetrahedrons[tetrahedronCount++] = { v0, v2, v5, v1 };
						blockTetrahedrons[tetrahedronCount++] = { v7, v2, v0, v3 };
						blockTetrahedrons[tetrahedronCount++] = { v5, v2, v7, v6 };
						blockTetrahedrons[tetrahedronCount++] = { v7, v0, v5, v4 };
						blockTetrahedrons[tetrahedronCount++] = { v0, v2, v7, v5 };
					}
				}
			}
		}

		B3_ASSERT(tetrahedronCount == 5 * W * H * D);

		vertices = blockVertices;
		tetrahedrons = blockTetrahedrons;
	}
};

#endif