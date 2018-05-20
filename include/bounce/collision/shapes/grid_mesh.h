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

#ifndef B3_GRID_MESH_H
#define B3_GRID_MESH_H

#include <bounce/collision/shapes/mesh.h>

// A (H + 1) x (W + 1) grid mesh stored in row-major order.
template<u32 H = 1, u32 W = 1>
struct b3GridMesh : public b3Mesh
{
	b3Vec3 gridVertices[ (H + 1) * (W + 1) ];
	b3Triangle gridTriangles[2 * H * W];

	// Set this grid to a W (width) per H (height) dimensioned grid centered at the origin and aligned
	// with the world x-z axes.
	b3GridMesh()
	{
		u32 h = H + 1;
		u32 w = W + 1;

		b3Vec3 t;
		t.x = -0.5f * float32(w) + 0.5f;
		t.y = 0.0f;
		t.z = -0.5f * float32(h) + 0.5f;

		for (u32 i = 0; i < h; ++i)
		{
			for (u32 j = 0; j < w; ++j)
			{
				u32 v1 = i * w + j;

				b3Vec3 v;
				v.x = float32(j);
				v.y = 0.0f;
				v.z = float32(i);

				v += t;

				gridVertices[v1] = v;
			}
		}

		u32 triangleIndex = 0;
		for (u32 i = 0; i < h - 1; ++i)
		{
			for (u32 j = 0; j < w - 1; ++j)
			{
				u32 v1 = i * w + j;
				u32 v2 = (i + 1) * w + j;
				u32 v3 = (i + 1) * w + (j + 1);
				u32 v4 = i * w + (j + 1);

				b3Triangle* t1 = gridTriangles + triangleIndex;
				++triangleIndex;

				t1->v1 = v3;
				t1->v2 = v2;
				t1->v3 = v1;

				b3Triangle* t2 = gridTriangles + triangleIndex;
				++triangleIndex;

				t2->v1 = v1;
				t2->v2 = v4;
				t2->v3 = v3;
			}
		}

		vertices = gridVertices;
		vertexCount = (H + 1) * (W + 1);
		triangles = gridTriangles;
		triangleCount = 2 * H * W;
	}
};

#endif