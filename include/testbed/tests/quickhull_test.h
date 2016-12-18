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

#ifndef QHULL_H
#define QHULL_H

#include <bounce\quickhull\qh_hull.h>

extern DebugDraw* g_debugDraw;
extern Camera g_camera;
extern Settings g_settings;

struct Pair
{
	void* key;
	u8 value;
};

struct Map
{
	void Add(const Pair& pair)
	{
		m_pairs.PushBack(pair);
	}

	Pair* Find(void* key)
	{
		for (u32 i = 0; i < m_pairs.Count(); ++i)
		{
			Pair* pair = m_pairs.Get(i);
			if (pair->key == key)
			{
				return pair;
			}
		}
		return nullptr;
	}

	b3StackArray<Pair, 256> m_pairs;
};

#define NULL_FEATURE 0xFF

inline b3Hull ConvertHull(const qhHull& hull)
{
	u8 V = 0;
	u8 E = 0;
	u8 F = 0;

	qhFace* face = hull.m_faceList.head;
	while (face)
	{
		qhHalfEdge* e = face->edge;
		do
		{
			++E;
			++V;
			e = e->next;
		} while (e != face->edge);

		++F;
		face = face->next;
	}

	u8 vertexCount = 0;
	b3Vec3* vertices = (b3Vec3*)b3Alloc(V * sizeof(b3Vec3));
	u8 edgeCount = 0;
	b3HalfEdge* edges = (b3HalfEdge*)b3Alloc(E * sizeof(b3HalfEdge));
	u8 faceCount = 0;
	b3Face* faces = (b3Face*)b3Alloc(F * sizeof(b3Face));
	b3Plane* planes = (b3Plane*)b3Alloc(F * sizeof(b3Plane));

	Map vertexMap;
	Map edgeMap;

	face = hull.m_faceList.head;
	while (face)
	{
		B3_ASSERT(faceCount < F);
		u8 iface = faceCount;
		b3Face* f = faces + faceCount;
		b3Plane* plane = planes + faceCount;
		++faceCount;

		*plane = face->plane;

		b3StackArray<u8, 32> faceEdges;

		qhHalfEdge* edge = face->edge;
		do
		{
			qhHalfEdge* twin = edge->twin;
			qhVertex* v1 = edge->tail;
			qhVertex* v2 = twin->tail;

			Pair* mte = edgeMap.Find(edge);
			Pair* mv1 = vertexMap.Find(v1);
			Pair* mv2 = vertexMap.Find(v2);

			u8 iv1;
			if (mv1)
			{
				iv1 = mv1->value;
			}
			else
			{
				B3_ASSERT(vertexCount < V);
				iv1 = vertexCount;
				vertices[iv1] = v1->position;
				vertexMap.Add({ v1, iv1 });
				++vertexCount;
			}

			u8 iv2;
			if (mv2)
			{
				iv2 = mv2->value;
			}
			else
			{
				B3_ASSERT(vertexCount < V);
				iv2 = vertexCount;
				vertices[iv2] = v2->position;
				vertexMap.Add({ v2, iv2 });
				++vertexCount;
			}

			if (mte)
			{
				u8 ie2 = mte->value;
				b3HalfEdge* e2 = edges + ie2;
				B3_ASSERT(e2->face == NULL_FEATURE);
				e2->face = iface;
				faceEdges.PushBack(ie2);
			}
			else
			{
				B3_ASSERT(edgeCount < E);
				u8 ie1 = edgeCount;
				b3HalfEdge* e1 = edges + edgeCount;
				++edgeCount;

				B3_ASSERT(edgeCount < E);
				u8 ie2 = edgeCount;
				b3HalfEdge* e2 = edges + edgeCount;
				++edgeCount;

				e1->face = iface;
				e1->origin = iv1;
				e1->twin = ie2;

				e2->face = NULL_FEATURE;
				e2->origin = iv2;
				e2->twin = ie1;

				faceEdges.PushBack(ie1);

				edgeMap.Add({ edge, ie1 } );
				edgeMap.Add({ twin, ie2 } );
			}

			edge = edge->next;
		} while (edge != face->edge);

		f->edge = faceEdges[0];
		for (u32 i = 0; i < faceEdges.Count(); ++i)
		{
			u32 j = i < faceEdges.Count() - 1 ? i + 1 : 0;
			edges[faceEdges[i]].next = faceEdges[j];
		}

		face = face->next;
	}

	b3Hull out;
	out.vertexCount = vertexCount;
	out.vertices = vertices;
	out.edgeCount = edgeCount;
	out.edges = edges;
	out.faceCount = faceCount;
	out.faces = faces;
	out.planes = planes;
	out.centroid.SetZero();

	for (u32 i = 0; i < vertexCount; ++i)
	{
		out.centroid += vertices[i];
	}
	out.centroid /= float32(vertexCount);

	out.Validate();
	return out;
}

inline void ConstructCylinder(b3Array<b3Vec3>& points, float32 radius = 1.0f, float32 height = 1.0f)
{
	{
		b3Vec3 center(0.0f, 0.0f, 0.0f);

		b3Vec3 normal;
		normal.Set(0.0f, 1.0f, 0.0f);

		u32 kEdgeCount = 20;
		float32 kAngleInc = 2.0f * B3_PI / float32(kEdgeCount);
		float32 cosInc = cos(kAngleInc);
		float32 sinInc = sin(kAngleInc);
		float32 tInc = 1.0f - cosInc;

		b3Vec3 n1 = b3Perp(normal);
		b3Vec3 v1 = center + radius * n1;
		for (u32 i = 0; i < kEdgeCount; ++i)
		{
			// Rodrigues' rotation formula
			b3Vec3 n2 = cosInc * n1 + sinInc * b3Cross(normal, n1) + tInc * b3Dot(normal, n1) * normal;
			b3Vec3 v2 = center + radius * n2;

			points.PushBack(v1);
			points.PushBack(v2);

			n1 = n2;
			v1 = v2;
		}
	}

	{
		b3Vec3 center(0.0f, height, 0.0f);

		b3Vec3 normal;
		normal.Set(0.0f, 1.0f, 0.0f);

		u32 kEdgeCount = 20;
		float32 kAngleInc = 2.0f * B3_PI / float32(kEdgeCount);
		float32 cosInc = cos(kAngleInc);
		float32 sinInc = sin(kAngleInc);
		float32 tInc = 1.0f - cosInc;

		b3Vec3 n1 = b3Perp(normal);
		b3Vec3 v1 = center + radius * n1;
		for (u32 i = 0; i < kEdgeCount; ++i)
		{
			// Rodrigues' rotation formula
			b3Vec3 n2 = cosInc * n1 + sinInc * b3Cross(normal, n1) + tInc * b3Dot(normal, n1) * normal;
			b3Vec3 v2 = center + radius * n2;

			points.PushBack(v1);
			points.PushBack(v2);

			n1 = n2;
			v1 = v2;
		}
	}
}

inline void ConstructCone(b3Array<b3Vec3>& points, float32 radius = 1.0f, float32 height = 1.0f)
{
	{
		b3Vec3 center(0.0f, 0.0f, 0.0f);

		b3Vec3 normal;
		normal.Set(0.0f, 1.0f, 0.0f);

		u32 kEdgeCount = 20;
		float32 kAngleInc = 2.0f * B3_PI / float32(kEdgeCount);
		float32 cosInc = cos(kAngleInc);
		float32 sinInc = sin(kAngleInc);
		float32 tInc = 1.0f - cosInc;

		b3Vec3 n1 = b3Perp(normal);
		b3Vec3 v1 = center + radius * n1;
		for (u32 i = 0; i < kEdgeCount; ++i)
		{
			// Rodrigues' rotation formula
			b3Vec3 n2 = cosInc * n1 + sinInc * b3Cross(normal, n1) + tInc * b3Dot(normal, n1) * normal;
			b3Vec3 v2 = center + radius * n2;

			points.PushBack(v1);
			points.PushBack(v2);

			n1 = n2;
			v1 = v2;
		}
	}

	b3Vec3 c(0.0f, height, 0.0f);
	points.PushBack(c);
}

class QuickhullTest : public Test
{
public:
	QuickhullTest()
	{
		g_camera.m_zoom = 15.0f;
		g_camera.m_q = b3Quat(b3Vec3(0.0f, 1.0f, 0.0f), 0.15f * B3_PI);
		g_camera.m_q = g_camera.m_q * b3Quat(b3Vec3(1.0f, 0.0f, 0.0f), -0.15f * B3_PI);
		g_camera.m_center.SetZero();

		b3BoxHull box;
		box.SetIdentity();

		b3StackArray<b3Vec3, 256> tetra;
		b3Vec3 v1(-1.0f, 0.0f, 0.0f);
		b3Vec3 v2(1.0f, 0.0f, 0.0f);
		b3Vec3 v3(0.0f, 0.0f, -1.0f);
		b3Vec3 v4 = 0.5f * (v1 + v2 + v3);
		v4.y += 2.0f;

		tetra.PushBack(v1);
		tetra.PushBack(v2);
		tetra.PushBack(v3);
		tetra.PushBack(v4);

		// Minkowski sum of box and tetrahedron
		b3StackArray<b3Vec3, 256> points;
		for (u32 i = 0; i < box.vertexCount; ++i)
		{
			for (u32 j = 0; j < tetra.Count(); ++j)
			{
				b3Vec3 p = box.vertices[i] - tetra[j];
				points.PushBack(p);
			}
		}

		u32 size = qhGetMemorySize(points.Count());
		m_memory = b3Alloc(size);
		m_qhull.Construct(m_memory, points);
	}

	~QuickhullTest()
	{
		b3Free(m_memory);
	}

	void Step()
	{
		m_qhull.Draw(g_debugDraw);
	}

	void KeyDown(int button)
	{
		if (button == GLFW_KEY_LEFT)
		{
			//m_index = b3Max(m_index - 1, 0);
		}

		if (button == GLFW_KEY_RIGHT)
		{
			//m_index = b3Min(m_index + 1, i32(horizon.Count()) - 1);
		}

		if (button == GLFW_KEY_I)
		{
		}
	}

	static Test* Create()
	{
		return new QuickhullTest();
	}

	void* m_memory;
	qhHull m_qhull;
};

#endif