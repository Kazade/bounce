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

#include <bounce/meshgen/cylinder_mesh.h>
#include <bounce/common/math/quat.h>

void cymCreateMesh(cymMesh& output, u32 segments)
{
	B3_ASSERT(segments > 2);

	u32 vertexCount = 2 * segments;
	b3Vec3* vertices = (b3Vec3*)b3Alloc(vertexCount * sizeof(b3Vec3));
	b3Vec3* normals = (b3Vec3*)b3Alloc(vertexCount * sizeof(b3Vec3));
	u32 indexCount = 3 * (2 * segments) + 2 * 3 * (segments - 2);
	u32* indices = (u32*)b3Alloc(indexCount * sizeof(u32));

	scalar angle = scalar(2) * B3_PI / scalar(segments);
	b3Quat q;
	q.SetAxisAngle(b3Vec3_y, angle);

	// Lower 
	b3Vec3 v(scalar(1), scalar(-0.5), scalar(0));
	for (u32 i = 0; i < segments; ++i)
	{
		vertices[i] = v;
		v = b3Mul(q, v);
	}

	// Upper 
	v.Set(scalar(1), scalar(0.5), scalar(0));
	for (u32 i = 0; i < segments; ++i)
	{
		vertices[segments + i] = v;
		v = b3Mul(q, v);
	}

	u32 idx = 0;

	// Side triangles
	for (u32 i = 0; i < segments; ++i)
	{
		u32 i1 = i;
		u32 i2 = i1 + 1 < segments ? i1 + 1 : 0;

		u32 i3 = segments + i;
		u32 i4 = i3 + 1 < 2 * segments ? i3 + 1 : segments;

		indices[idx++] = i1;
		indices[idx++] = i2;
		indices[idx++] = i4;

		indices[idx++] = i4;
		indices[idx++] = i3;
		indices[idx++] = i1;
	}

	// Side normals
	for (u32 i = 0; i < vertexCount; ++i)
	{
		normals[i].SetZero();
	}

	for (u32 i = 0; i < idx / 3; ++i)
	{
		u32 i1 = indices[3 * i + 0];
		u32 i2 = indices[3 * i + 1];
		u32 i3 = indices[3 * i + 2];

		b3Vec3 v1 = vertices[i1];
		b3Vec3 v2 = vertices[i2];
		b3Vec3 v3 = vertices[i3];

		b3Vec3 n = b3Cross(v2 - v1, v3 - v1);
		n.Normalize();

		normals[i1] += n;
		normals[i2] += n;
		normals[i3] += n;
	}
	
	for (u32 i = 0; i < vertexCount; ++i)
	{
		normals[i].Normalize();
	}

	// Lower. Reverse loop to ensure CCW 
	u32 i1 = segments - 1;
	for (u32 i2 = i1 - 1; i2 > 0; --i2)
	{
		u32 i3 = i2 - 1;

		indices[idx++] = i1;
		indices[idx++] = i2;
		indices[idx++] = i3;
	}

	// Upper
	i1 = segments;
	for (u32 i2 = i1 + 1; i2 < 2 * segments - 1; ++i2)
	{
		u32 i3 = i2 + 1;

		indices[idx++] = i1;
		indices[idx++] = i2;
		indices[idx++] = i3;
	}

	B3_ASSERT(idx == indexCount);

	output.vertexCount = vertexCount;
	output.vertices = vertices;
	output.normals = normals;
	output.indexCount = indexCount;
	output.indices = indices;
}