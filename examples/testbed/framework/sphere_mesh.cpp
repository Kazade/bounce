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

#include <testbed/framework/sphere_mesh.h>

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

static inline void smSetAsIcosahedron(smMesh& mesh)
{
	assert(mesh.vertexCount == 0);

	float32 t = 0.5f * (1.0f + b3Sqrt(5.0f));

	smAddVertex(mesh, -1.0f, t, 0.0f);
	smAddVertex(mesh, 1.0f, t, 0.0f);
	smAddVertex(mesh, -1.0f, -t, 0.0f);
	smAddVertex(mesh, 1.0f, -t, 0.0f);

	smAddVertex(mesh, 0.0f, -1.0f, t);
	smAddVertex(mesh, 0.0f, 1.0f, t);
	smAddVertex(mesh, 0.0f, -1.0f, -t);
	smAddVertex(mesh, 0.0f, 1.0f, -t);

	smAddVertex(mesh, t, 0.0f, -1.0f);
	smAddVertex(mesh, t, 0.0f, 1.0f);
	smAddVertex(mesh, -t, 0.0f, -1.0f);
	smAddVertex(mesh, -t, 0.0f, 1.0f);

	for (u32 i = 0; i < mesh.vertexCount; ++i)
	{
		mesh.vertices[i].Normalize();
	}

	assert(mesh.indexCount == 0);

	smAddTriangle(mesh, 0, 11, 5);
	smAddTriangle(mesh, 0, 5, 1);
	smAddTriangle(mesh, 0, 1, 7);
	smAddTriangle(mesh, 0, 7, 10);
	smAddTriangle(mesh, 0, 10, 11);

	smAddTriangle(mesh, 1, 5, 9);
	smAddTriangle(mesh, 5, 11, 4);
	smAddTriangle(mesh, 11, 10, 2);
	smAddTriangle(mesh, 10, 7, 6);
	smAddTriangle(mesh, 7, 1, 8);

	smAddTriangle(mesh, 3, 9, 4);
	smAddTriangle(mesh, 3, 4, 2);
	smAddTriangle(mesh, 3, 2, 6);
	smAddTriangle(mesh, 3, 6, 8);
	smAddTriangle(mesh, 3, 8, 9);

	smAddTriangle(mesh, 4, 9, 5);
	smAddTriangle(mesh, 2, 4, 11);
	smAddTriangle(mesh, 6, 2, 10);
	smAddTriangle(mesh, 8, 6, 7);
	smAddTriangle(mesh, 9, 8, 1);
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

static inline smEdgeVertexPair* smFindOpposite(smEdgeVertexMap& map, const smEdge& edge)
{
	for (u32 i = 0; i < map.pairCount; ++i)
	{
		smEdgeVertexPair* pair = map.pairs + i;

		if (pair->edge.v1 == edge.v2 && pair->edge.v2 == edge.v1)
		{
			return pair;
		}
	}
	return nullptr;
}

static inline u32 smSubdivideEdge(smMesh& in_out, smEdgeVertexMap& map, 
	u32 i1, u32 i2)
{
	smEdge edge;
	edge.v1 = i1;
	edge.v2 = i2;

	smEdgeVertexPair* pair = smFindOpposite(map, edge);

	if (pair)
	{
		return pair->vertex;
	}

	smEdge newEdge;
	newEdge.v1 = i1;
	newEdge.v2 = i2;
	
	u32 newVertex = in_out.vertexCount;

	b3Vec3 v1 = in_out.vertices[i1];
	b3Vec3 v2 = in_out.vertices[i2];

	b3Vec3 v = 0.5f * (v1 + v2);
	float32 len = v.Normalize();

	smAddVertex(in_out, v.x, v.y, v.z);

	smEdgeVertexPair newPair;
	newPair.edge = newEdge;
	newPair.vertex = newVertex;

	smAddPair(map, newPair);

	return newVertex;
}

static void smSubdivideMesh(smMesh& in_out, smEdgeVertexMap& map)
{
	map.pairCount = 0;

	u32 inputIndexCount = in_out.indexCount;

	for (u32 i = 0; i < inputIndexCount / 3; ++i)
	{
		u32 vi1 = in_out.indices[3 * i + 0];
		u32 vi2 = in_out.indices[3 * i + 1];
		u32 vi3 = in_out.indices[3 * i + 2];
		
		u32 vi4 = smSubdivideEdge(in_out, map, vi1, vi2);
		u32 vi5 = smSubdivideEdge(in_out, map, vi2, vi3);
		u32 vi6 = smSubdivideEdge(in_out, map, vi3, vi1);

		smAddTriangle(in_out, vi1, vi4, vi6);
		smAddTriangle(in_out, vi4, vi2, vi5);
		smAddTriangle(in_out, vi5, vi3, vi6);
		smAddTriangle(in_out, vi4, vi5, vi6);
	}
}

// Compute the maximum number of vertices and 
// the number of triangle vertex indices in the intermediate mesh.
// Also compute the maximum number of edge-vertex pairs per subdivision step 
static inline void smCount(u32& vertexCount, u32& indexCount, u32& edgeVertexPairCount, u32 subdivisions)
{
	vertexCount = 12;
	indexCount = 3 * 20;
	for (u32 i = 0; i < subdivisions; ++i)
	{
		vertexCount += 3 * (indexCount / 3);
		indexCount += 4 * 3 * (indexCount / 3);
	}
	edgeVertexPairCount = 3 * (indexCount / 3);
}

void smCreateMesh(smMesh& output, u32 subdivisions)
{
	assert(output.vertexCount == 0);
	assert(output.indexCount == 0);

	u32 vertexCount, indexCount, edgeVertexPairCount;
	smCount(vertexCount, indexCount, edgeVertexPairCount, subdivisions);

	u32 byteCount = 0;
	byteCount += vertexCount * sizeof(b3Vec3);
	byteCount += indexCount * sizeof(u32);
	byteCount += edgeVertexPairCount * sizeof(smEdgeVertexPair);

	u8* bytes = (u8*)malloc(byteCount);

	smMesh out;
	out.vertexCount = 0;
	out.vertices = (b3Vec3*)bytes; 
	out.indexCount = 0;
	out.indices = (u32*) ((u8*)(out.vertices) + (vertexCount * sizeof(b3Vec3)));

	smEdgeVertexMap map;
	map.pairCount = 0;
	map.pairs = (smEdgeVertexPair*) ((u8*)(out.indices) + (indexCount * sizeof(u32)));

	smSetAsIcosahedron(out);

	for (u32 i = 0; i < subdivisions; ++i)
	{
		smSubdivideMesh(out, map);
	}

	assert(map.pairCount < edgeVertexPairCount);
	
	assert(out.vertexCount < vertexCount);
	assert(out.indexCount == indexCount);

	output.vertexCount = out.vertexCount;
	output.vertices = (b3Vec3*)malloc(out.vertexCount * sizeof(b3Vec3));
	memcpy(output.vertices, out.vertices, out.vertexCount * sizeof(b3Vec3));

	output.indexCount = out.indexCount;
	output.indices = (u32*)malloc(out.indexCount * sizeof(u32));
	memcpy(output.indices, out.indices, out.indexCount * sizeof(u32));

	free(bytes);

	out.vertices = nullptr;
	out.indices = nullptr;
}