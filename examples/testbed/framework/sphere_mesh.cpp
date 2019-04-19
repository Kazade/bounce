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

// References:
// https://github.com/caosdoar/spheres
// https://schneide.blog/2016/07/15/generating-an-icosphere-in-c/

struct smEdge
{
	smEdge() { }

	smEdge(u32 _v1, u32 _v2)
	{
		v1 = _v1;
		v2 = _v2;
	}

	u32 v1, v2;
};

struct smEdgeVertexPair
{
	smEdgeVertexPair() { }

	smEdgeVertexPair(const smEdge& _edge, u32 _vertex)
	{
		edge = _edge;
		vertex = _vertex;
	}

	smEdge edge;
	u32 vertex;
};

struct smEdgeVertexMap
{
	smEdgeVertexMap() { }

	smEdgeVertexPair* Find(const smEdge& edge)
	{
		for (u32 i = 0; i < pairs.Count(); ++i)
		{
			smEdgeVertexPair* pair = pairs.Get(i);

			if ((pair->edge.v1 == edge.v1 && pair->edge.v2 == edge.v2) || 
				(pair->edge.v1 == edge.v2 && pair->edge.v2 == edge.v1) )
			{
				return pair;
			}
		}
		return nullptr;
	}

	b3StackArray<smEdgeVertexPair, 32> pairs;
};

// 
static inline u32 smSubdivideEdge(u32 i1, u32 i2,
	const b3Vec3& v1, const b3Vec3& v2,
	smEdgeVertexMap& edgeVertexMap,
	smMesh& output)
{
	smEdge edge(i1, i2);

	smEdgeVertexPair* pair = edgeVertexMap.Find(edge);

	if (pair)
	{
		return pair->vertex;
	}

	smEdge newEdge(i1, i2);
	u32 newVertex = output.vertices.Count();

	b3Vec3 v = 0.5f * (v1 + v2);
	float32 len = v.Normalize();

	output.AddVertex(v.x, v.y, v.z);

	smEdgeVertexPair newPair(newEdge, newVertex);

	edgeVertexMap.pairs.PushBack(newPair);

	return newVertex;
}

// 
static inline void smSubdivideMesh(smMesh& output, const smMesh& input)
{
	output.vertices = input.vertices;

	smEdgeVertexMap edgeVertexMap;

	for (u32 i = 0; i < input.triangleIndices.Count() / 3; ++i)
	{
		u32 vi1 = input.triangleIndices[3 * i + 0];
		u32 vi2 = input.triangleIndices[3 * i + 1];
		u32 vi3 = input.triangleIndices[3 * i + 2];

		b3Vec3 v1 = input.vertices[vi1];
		b3Vec3 v2 = input.vertices[vi2];
		b3Vec3 v3 = input.vertices[vi3];

		u32 vi4 = smSubdivideEdge(vi1, vi2, v1, v2, edgeVertexMap, output);
		u32 vi5 = smSubdivideEdge(vi2, vi3, v2, v3, edgeVertexMap, output);
		u32 vi6 = smSubdivideEdge(vi3, vi1, v3, v1, edgeVertexMap, output);

		output.AddTriangle(vi1, vi4, vi6);
		output.AddTriangle(vi4, vi2, vi5);
		output.AddTriangle(vi5, vi3, vi6);
		output.AddTriangle(vi4, vi5, vi6);
	}
}

//
void smCreateMesh(smMesh& output, u32 subdivisions)
{
	assert(output.vertices.Count() == 0);
	assert(output.triangleIndices.Count() == 0);

	smMesh input;
	input.SetAsIcosahedron();

	for (u32 i = 0; i < subdivisions; ++i)
	{
		smMesh subOutput;
		smSubdivideMesh(subOutput, input);
		input = subOutput;
	}

	output = input;
}
