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

#include <bounce/meshgen/sphere_mesh.h>

static inline void smAddVertex(smMesh& mesh, float32 x, float32 y, float32 z)
{
	mesh.vertices[mesh.vertexCount++].Set(x, y, z);
}

static inline void smAddTriangle(smMesh& mesh, u32 v1, u32 v2, u32 v3)
{
	mesh.indices[mesh.indexCount++] = v1;
	mesh.indices[mesh.indexCount++] = v2;
	mesh.indices[mesh.indexCount++] = v3;
}

static inline void smSetAsOctahedron(smMesh& mesh)
{
	B3_ASSERT(mesh.vertexCount == 0);

	smAddVertex(mesh, 0.0f, -1.0f, 0.0f);
	smAddVertex(mesh, 0.0f, 0.0f, 1.0f);
	smAddVertex(mesh, -1.0f, 0.0f, 0.0f);
	smAddVertex(mesh, 0.0f, 0.0f, -1.0f);
	smAddVertex(mesh, 1.0f, 0.0f, 0.0f);
	smAddVertex(mesh, 0.0f, 1.0f, 0.0f);

	B3_ASSERT(mesh.indexCount == 0);

	smAddTriangle(mesh, 0, 1, 2);
	smAddTriangle(mesh, 0, 2, 3);
	smAddTriangle(mesh, 0, 3, 4);
	smAddTriangle(mesh, 0, 4, 1);
	
	smAddTriangle(mesh, 5, 2, 1);
	smAddTriangle(mesh, 5, 3, 2);
	smAddTriangle(mesh, 5, 4, 3);
	smAddTriangle(mesh, 5, 1, 4);
}

struct smEdge
{
	u32 v1, v2;
};

struct smEdgeVertexPair
{
	smEdge edge;
	u32 vertex;
};

struct smEdgeVertexMap
{
	u32 pairCount;
	smEdgeVertexPair* pairs;
};

static inline void smAddPair(smEdgeVertexMap& map, const smEdgeVertexPair& pair)
{
	map.pairs[map.pairCount++] = pair;
}

static inline smEdgeVertexPair* smFind(smEdgeVertexMap& map, u32 v1, u32 v2)
{
	for (u32 i = 0; i < map.pairCount; ++i)
	{
		smEdgeVertexPair* pair = map.pairs + i;

		if (pair->edge.v1 == v1 && pair->edge.v2 == v2)
		{
			return pair;
		}
	}
	return nullptr;
}

static inline u32 smSubdivideEdge(smMesh& out, smEdgeVertexMap& map, 
	const smMesh& in, 
	u32 i1, u32 i2)
{
	smEdgeVertexPair* pair = smFind(map, i2, i1);

	if (pair)
	{
		return pair->vertex;
	}

	smEdge newEdge;
	newEdge.v1 = i1;
	newEdge.v2 = i2;

	u32 newVertex = out.vertexCount;

	b3Vec3 v1 = in.vertices[i1];
	b3Vec3 v2 = in.vertices[i2];
	b3Vec3 v = 0.5f * (v1 + v2);
	v.Normalize();

	smAddVertex(out, v.x, v.y, v.z);

	smEdgeVertexPair newPair;
	newPair.edge = newEdge;
	newPair.vertex = newVertex;

	smAddPair(map, newPair);

	return newVertex;
}

static void smSubdivideMesh(smMesh& out, const smMesh& in, smEdgeVertexMap& map)
{
	B3_ASSERT(out.vertexCount == 0);
	B3_ASSERT(out.indexCount == 0);
	B3_ASSERT(map.pairCount == 0);

	out.vertexCount = in.vertexCount;
	memcpy(out.vertices, in.vertices, in.vertexCount * sizeof(b3Vec3));

	for (u32 i = 0; i < in.indexCount / 3; ++i)
	{
		u32 vi1 = in.indices[3 * i + 0];
		u32 vi2 = in.indices[3 * i + 1];
		u32 vi3 = in.indices[3 * i + 2];
		
		u32 vi4 = smSubdivideEdge(out, map, in, vi1, vi2);
		u32 vi5 = smSubdivideEdge(out, map, in, vi2, vi3);
		u32 vi6 = smSubdivideEdge(out, map, in, vi3, vi1);

		smAddTriangle(out, vi1, vi4, vi6);
		smAddTriangle(out, vi4, vi2, vi5);
		smAddTriangle(out, vi5, vi3, vi6);
		smAddTriangle(out, vi4, vi5, vi6);
	}
}

void smCreateMesh(smMesh& output, u32 subdivisions)
{
	B3_ASSERT(output.vertexCount == 0);
	B3_ASSERT(output.indexCount == 0);

	smMesh in;
	in.vertexCount = 0;
	in.vertices = (b3Vec3*)b3Alloc(6 * sizeof(b3Vec3));
	in.normals = nullptr;
	in.indexCount = 0;
	in.indices = (u32*)b3Alloc(3 * 8 * sizeof(u32));

	smSetAsOctahedron(in);

	for (u32 i = 0; i < subdivisions; ++i)
	{
		u32 inTriangleCount = in.indexCount / 3;
		u32 outVertexCapacity = in.vertexCount + 3 * inTriangleCount;
		
		u32 outTriangleCapacity = 4 * inTriangleCount;
		u32 outIndexCapacity = 3 * outTriangleCapacity;

		u32 edgeVertexPairCapacity = 3 * inTriangleCount;

		smMesh out;
		out.vertexCount = 0;
		out.vertices = (b3Vec3*)b3Alloc(outVertexCapacity * sizeof(b3Vec3));

		out.indexCount = 0;
		out.indices = (u32*)b3Alloc(outIndexCapacity * sizeof(u32));

		smEdgeVertexMap map;
		map.pairCount = 0;
		map.pairs = (smEdgeVertexPair*)b3Alloc(edgeVertexPairCapacity * sizeof(smEdgeVertexPair));

		smSubdivideMesh(out, in, map);

		b3Free(map.pairs);
		b3Free(in.vertices);
		b3Free(in.indices);
		
		// in = out
		in.vertexCount = out.vertexCount;
		in.vertices = (b3Vec3*)b3Alloc(out.vertexCount * sizeof(b3Vec3));
		memcpy(in.vertices, out.vertices, out.vertexCount * sizeof(b3Vec3));

		in.indexCount = out.indexCount;
		in.indices = (u32*)b3Alloc(out.indexCount * sizeof(u32));
		memcpy(in.indices, out.indices, out.indexCount * sizeof(u32));

		b3Free(out.vertices);
		out.vertices = nullptr;
		out.normals = nullptr;
		b3Free(out.indices);
		out.indices = nullptr;
	}

	output.vertexCount = in.vertexCount;
	output.vertices = (b3Vec3*)b3Alloc(in.vertexCount * sizeof(b3Vec3));
	memcpy(output.vertices, in.vertices, in.vertexCount * sizeof(b3Vec3));
	
	output.normals = (b3Vec3*)b3Alloc(in.vertexCount * sizeof(b3Vec3));
	memcpy(output.normals, output.vertices, output.vertexCount * sizeof(b3Vec3));

	output.indexCount = in.indexCount;
	output.indices = (u32*)b3Alloc(in.indexCount * sizeof(u32));
	memcpy(output.indices, in.indices, in.indexCount * sizeof(u32));

	b3Free(in.vertices);
	b3Free(in.indices);
	in.vertices = nullptr;
	in.normals = nullptr;
	in.indices = nullptr;
}