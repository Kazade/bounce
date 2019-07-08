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

static inline void smCount(u32& inVertexCapacity, u32& inIndexCount, 
	u32& outVertexCapacity, u32& outIndexCount, 
	u32& edgeVertexPairCapacity, 
	u32 subdivisions)
{
	inVertexCapacity = 6;
	u32 inTriangleCount = 8;
	
	outVertexCapacity = 0;
	u32 outTriangleCount = 0;

	edgeVertexPairCapacity = 0;

	for (u32 i = 0; i < subdivisions; ++i)
	{
		outVertexCapacity = inVertexCapacity + 3 * inTriangleCount;
		outTriangleCount = 4 * inTriangleCount;

		edgeVertexPairCapacity = 3 * inTriangleCount;
		
		inVertexCapacity = outVertexCapacity;
		inTriangleCount = outTriangleCount;
	}
	
	inIndexCount = 3 * inTriangleCount;
	outIndexCount = 3 * outTriangleCount;
}

void smCreateMesh(smMesh& output, u32 subdivisions)
{
	B3_ASSERT(output.vertexCount == 0);
	B3_ASSERT(output.indexCount == 0);

	u32 inVertexCapacity, inIndexCount;
	u32 outVertexCapacity, outIndexCount;
	u32 edgeVertexPairCapacity;
	smCount(inVertexCapacity, inIndexCount, outVertexCapacity, outIndexCount, edgeVertexPairCapacity, subdivisions);

	u32 byteCount = 0;
	byteCount += inVertexCapacity * sizeof(b3Vec3);
	byteCount += inIndexCount * sizeof(u32);
	byteCount += outVertexCapacity * sizeof(b3Vec3);
	byteCount += outIndexCount * sizeof(u32);
	byteCount += edgeVertexPairCapacity * sizeof(smEdgeVertexPair);
	
	u8* bytes = (u8*)b3Alloc(byteCount);

	b3Vec3* inVertex = (b3Vec3*)bytes;
	u32* inIndex = (u32*)((u8*)(inVertex) + inVertexCapacity * sizeof(b3Vec3));
	b3Vec3* outVertex = (b3Vec3*) ((u8*)(inIndex) + inIndexCount * sizeof(u32));
	u32* outIndex = (u32*)((u8*)(outVertex) + outVertexCapacity * sizeof(b3Vec3));
	smEdgeVertexPair* pairs = (smEdgeVertexPair*)((u8*)(outIndex) + outIndexCount * sizeof(u32));

	smMesh in;
	in.vertexCount = 0;
	in.vertices = inVertex;
	in.normals = nullptr;
	in.indexCount = 0;
	in.indices = inIndex;

	smSetAsOctahedron(in);

	for (u32 i = 0; i < subdivisions; ++i)
	{
		smMesh out;
		out.vertexCount = 0;
		out.vertices = outVertex;
		out.indexCount = 0;
		out.indices = outIndex;

		smEdgeVertexMap map;
		map.pairCount = 0;
		map.pairs = pairs;

		smSubdivideMesh(out, in, map);

		// in = out
		in.vertexCount = out.vertexCount;
		in.vertices = inVertex;
		memcpy(in.vertices, out.vertices, out.vertexCount * sizeof(b3Vec3));

		in.indexCount = out.indexCount;
		in.indices = inIndex;
		memcpy(in.indices, out.indices, out.indexCount * sizeof(u32));

		out.vertices = nullptr;
		out.normals = nullptr;
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

	b3Free(bytes);

	in.vertices = nullptr;
	in.normals = nullptr;
	in.indices = nullptr;
}