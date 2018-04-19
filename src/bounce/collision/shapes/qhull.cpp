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

#include <bounce/collision/shapes/qhull.h>
#include <bounce/quickhull/qh_hull.h>

#define B3_NULL_HULL_FEATURE 0xFF

//
struct b3PointerIndex
{
	void* key;
	u8 value;
};

// 
template<u32 N>
struct b3PointerMap
{
	void Add(const b3PointerIndex& entry)
	{
		m_entries.PushBack(entry);
	}

	b3PointerIndex* Find(void* key)
	{
		for (u32 i = 0; i < m_entries.Count(); ++i)
		{
			b3PointerIndex* index = m_entries.Get(i);
			if (index->key == key)
			{
				return index;
			}
		}
		return NULL;
	}
	
	b3StackArray<b3PointerIndex, N> m_entries;
};

//
static b3Vec3 b3ComputeCentroid(b3QHull* hull)
{
	B3_ASSERT(hull->vertexCount >= 3);

	// volume = int(dV)
	float32 volume = 0.0f;

	// centroid.x = (1 / volume) * int(x * dV)
	// centroid.y = (1 / volume) * int(y * dV)
	// centroid.z = (1 / volume) * int(z * dV)
	b3Vec3 centroid; centroid.SetZero();
	
	// Put the reference point inside the hull
	b3Vec3 s; s.SetZero();
	for (u32 i = 0; i < hull->vertexCount; ++i)
	{
		s += hull->vertices[i];
	}
	s /= float32(hull->vertexCount);

	const float32 inv6 = 1.0f / 6.0f;
	const float32 inv12 = 1.0f / 12.0f;

	for (u32 i = 0; i < hull->faceCount; ++i)
	{
		const b3Face* face = hull->GetFace(i);
		const b3HalfEdge* begin = hull->GetEdge(face->edge);

		const b3HalfEdge* edge = hull->GetEdge(begin->next);
		do
		{
			const b3HalfEdge* next = hull->GetEdge(edge->next);

			u32 i1 = begin->origin;
			u32 i2 = edge->origin;
			u32 i3 = next->origin;

			b3Vec3 p1 = hull->GetVertex(i1) - s;
			b3Vec3 p2 = hull->GetVertex(i2) - s;
			b3Vec3 p3 = hull->GetVertex(i3) - s;

			float32 px1 = p1.x, py1 = p1.y, pz1 = p1.z;
			float32 px2 = p2.x, py2 = p2.y, pz2 = p2.z;
			float32 px3 = p3.x, py3 = p3.y, pz3 = p3.z;

			// 
			b3Vec3 D = b3Cross(p2 - p1, p3 - p1);
			float32 Dx = D.x, Dy = D.y, Dz = D.z;

			//
			float32 intx = px1 + px2 + px3;
			volume += (inv6 * D.x) * intx;

			//
			float32 intx2 = px1 * px1 + px1 * px2 + px1 * px3 + px2 * px2 + px2 * px3 + px3 * px3;
			float32 inty2 = py1 * py1 + py1 * py2 + py1 * py3 + py2 * py2 + py2 * py3 + py3 * py3;
			float32 intz2 = pz1 * pz1 + pz1 * pz2 + pz1 * pz3 + pz2 * pz2 + pz2 * pz3 + pz3 * pz3;

			centroid.x += (0.5f * inv12 * Dx) * intx2;
			centroid.y += (0.5f * inv12 * Dy) * inty2;
			centroid.z += (0.5f * inv12 * Dz) * intz2;

			edge = next;
		} while (hull->GetEdge(edge->next) != begin);
	}

	// Centroid
	B3_ASSERT(volume > B3_EPSILON);
	centroid /= volume;
	centroid += s;
	return centroid;
}

void b3QHull::Set(const b3Vec3* points, u32 count)
{
	// Copy points into local buffer, remove coincident points.
	b3StackArray<b3Vec3, B3_MAX_HULL_VERTICES> ps;
	for (u32 i = 0; i < count; ++i)
	{
		b3Vec3 p = points[i];

		bool unique = true;

		for (u32 j = 0; j < ps.Count(); ++j)
		{
			const float32 kTol = 0.5f * B3_LINEAR_SLOP;
			if (b3DistanceSquared(p, ps[j]) < kTol * kTol)
			{
				unique = false;
				break;
			}
		}

		if (unique)
		{
			ps.PushBack(p);
		}
	}

	if (ps.Count() < 3)
	{
		// Polyhedron is degenerate.
		return;
	}

	// Create a convex hull.
	u32 qh_size = qhGetMemorySize(ps.Count());
	void* qh_memory = b3Alloc(qh_size);

	qhHull hull;
	hull.Construct(qh_memory, ps);

	// Count vertices, edges, and faces in the convex hull.
	u32 V = 0;
	u32 E = 0;
	u32 F = 0;

	const qhList<qhFace>& faceList = hull.GetFaceList();

	qhFace* face = faceList.head;
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

	if (V > B3_MAX_HULL_VERTICES || E > B3_MAX_HULL_EDGES || F > B3_MAX_HULL_FACES)
	{
		b3Free(qh_memory);
		return; 
	}

	// Convert the constructed hull into a run-time hull.
	vertexCount = 0;
	edgeCount = 0;
	faceCount = 0;

	b3PointerMap<B3_MAX_HULL_VERTICES> vertexMap;
	b3PointerMap<B3_MAX_HULL_EDGES> edgeMap;

	face = faceList.head;
	while (face)
	{
		// Collected face half-edges
		b3StackArray<u8, 32> faceEdges;

		qhHalfEdge* edge = face->edge;
		do
		{
			qhHalfEdge* twin = edge->twin;
			
			qhVertex* v1 = edge->tail;
			qhVertex* v2 = twin->tail;

			b3PointerIndex* mv1 = vertexMap.Find(v1);
			b3PointerIndex* mv2 = vertexMap.Find(v2);

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
			
			b3PointerIndex* mte = edgeMap.Find(edge);
			
			if (mte)
			{
				u8 ie2 = mte->value;
				b3HalfEdge* e2 = edges + ie2;
				B3_ASSERT(e2->face == B3_NULL_HULL_FEATURE);
				e2->face = u8(faceCount);
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

				e1->face = u8(faceCount);
				e1->origin = iv1;
				e1->twin = ie2;

				e2->face = B3_NULL_HULL_FEATURE;
				e2->origin = iv2;
				e2->twin = ie1;

				faceEdges.PushBack(ie1);

				edgeMap.Add({ edge, ie1 });
				edgeMap.Add({ twin, ie2 });
			}

			edge = edge->next;
		} while (edge != face->edge);

		B3_ASSERT(faceEdges.Count() > 0);
		
		b3Face* f = faces + faceCount;
		f->edge = faceEdges[0];
		for (u32 i = 0; i < faceEdges.Count(); ++i)
		{
			u32 j = i < faceEdges.Count() - 1 ? i + 1 : 0;
			edges[faceEdges[i]].next = faceEdges[j];
		}

		planes[faceCount] = face->plane;
		
		B3_ASSERT(faceCount < F);
		++faceCount;

		face = face->next;
	}

	// 
	b3Free(qh_memory);

	// Validate
	Validate();

	// Compute the centroid.
	centroid = b3ComputeCentroid(this);
}

void b3QHull::SetAsCylinder(float32 radius, float32 height)
{
	B3_ASSERT(radius > 0.0f);
	B3_ASSERT(height > 0.0f);

	const u32 kEdgeCount = 20;
	const u32 kVertexCount = 4 * kEdgeCount;
	b3Vec3 vs[kVertexCount];

	u32 count = 0;

	float32 kAngleInc = 2.0f * B3_PI / float32(kEdgeCount);
	b3Quat q = b3QuatRotationY(kAngleInc);

	{
		b3Vec3 center(0.0f, 0.0f, 0.0f);
		b3Vec3 n1(1.0f, 0.0f, 0.0f);
		b3Vec3 v1 = center + radius * n1;
		for (u32 i = 0; i < kEdgeCount; ++i)
		{
			b3Vec3 n2 = b3Mul(q, n1);
			b3Vec3 v2 = center + radius * n2;

			vs[count++] = v1;
			vs[count++] = v2;

			n1 = n2;
			v1 = v2;
		}
	}

	{
		b3Vec3 center(0.0f, height, 0.0f);
		b3Vec3 n1(1.0f, 0.0f, 0.0f);
		b3Vec3 v1 = center + radius * n1;
		for (u32 i = 0; i < kEdgeCount; ++i)
		{
			b3Vec3 n2 = b3Mul(q, n1);
			b3Vec3 v2 = center + radius * n2;

			vs[count++] = v1;
			vs[count++] = v2;

			n1 = n2;
			v1 = v2;
		}
	}

	// Set
	Set(vs, count);
}

void b3QHull::SetAsCone(float32 radius, float32 height)
{
	B3_ASSERT(radius > 0.0f);
	B3_ASSERT(height > 0.0f);
	
	const u32 kEdgeCount = 20;
	const u32 kVertexCount = 2 * kEdgeCount + 1;
	b3Vec3 vs[kVertexCount];

	u32 count = 0;

	float32 kAngleInc = 2.0f * B3_PI / float32(kEdgeCount);
	b3Quat q = b3QuatRotationY(kAngleInc);

	b3Vec3 center(0.0f, 0.0f, 0.0f);
	b3Vec3 n1(1.0f, 0.0f, 0.0f);
	b3Vec3 v1 = center + radius * n1;
	for (u32 i = 0; i < kEdgeCount; ++i)
	{
		b3Vec3 n2 = b3Mul(q, n1);
		b3Vec3 v2 = center + radius * n2;

		vs[count++] = v1;
		vs[count++] = v2;

		n1 = n2;
		v1 = v2;
	}

	vs[count++].Set(0.0f, height, 0.0f);

	// Set
	Set(vs, count);
}