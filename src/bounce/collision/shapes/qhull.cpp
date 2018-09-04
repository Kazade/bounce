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

template <class T>
struct b3UniqueStackArray
{
	u32 PushBack(const T& e)
	{
		for (u32 i = 0; i < elements.Count(); ++i)
		{
			if (elements[i] == e)
			{
				return i;
			}
		}

		elements.PushBack(e);
		return elements.Count() - 1;
	}

	b3StackArray<T, 256> elements;
};

//
static b3Vec3 b3ComputeCentroid(b3QHull* hull)
{
	// M. Kallay - "Computing the Moment of Inertia of a Solid Defined by a Triangle Mesh"
	
	B3_ASSERT(hull->vertexCount >= 4);

	float32 volume = 0.0f;

	b3Vec3 centroid; centroid.SetZero();

	// Put the reference point inside the hull
	b3Vec3 s; s.SetZero();
	for (u32 i = 0; i < hull->vertexCount; ++i)
	{
		s += hull->vertices[i];
	}
	s /= float32(hull->vertexCount);

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

			b3Vec3 v1 = hull->GetVertex(i1) - s;
			b3Vec3 v2 = hull->GetVertex(i2) - s;
			b3Vec3 v3 = hull->GetVertex(i3) - s;

			// Signed tetrahedron volume
			float32 D = b3Det(v1, v2, v3);

			// Contribution to the mass
			volume += D;

			// Contribution to the centroid
			b3Vec3 v4 = v1 + v2 + v3;

			centroid += D * v4;

			edge = next;
		} while (hull->GetEdge(edge->next) != begin);
	}

	// Centroid
	B3_ASSERT(volume > B3_EPSILON);
	centroid /= 4.0f * volume;
	centroid += s;
	return centroid;
}

void b3QHull::Set(u32 vtxStride, const void* vtxBase, u32 vtxCount, bool simplify)
{
	B3_ASSERT(vtxStride >= sizeof(b3Vec3));
	B3_ASSERT(vtxCount >= 4);

	// Copy vertices into local buffer, perform welding.
	u32 vs0Count = 0;
	b3Vec3* vs0 = (b3Vec3*)b3Alloc(vtxCount * sizeof(b3Vec3));
	for (u32 i = 0; i < vtxCount; ++i)
	{
		b3Vec3 v = *(b3Vec3*)((u8*)vtxBase + vtxStride * i);

		B3_ASSERT(b3IsValid(v.x));
		B3_ASSERT(b3IsValid(v.y));
		B3_ASSERT(b3IsValid(v.z));

		bool unique = true;

		for (u32 j = 0; j < vs0Count; ++j)
		{
			if (b3DistanceSquared(v, vs0[j]) <= B3_LINEAR_SLOP * B3_LINEAR_SLOP)
			{
				unique = false;
				break;
			}
		}

		if (unique)
		{
			vs0[vs0Count++] = v;
		}
	}

	if (vs0Count < 4)
	{
		// Polyhedron is degenerate.
		b3Free(vs0);
		return;
	}

	// Create a convex hull.
	qhHull hull;

	if (simplify == true)
	{
		qhHull primary;
		primary.Construct(vs0, vs0Count);
		b3Free(vs0);

		// Simplify the constructed hull.

		// Put the origin inside the hull.
		b3Vec3 s;
		s.SetZero();
		for (qhVertex* v = primary.GetVertexList().head; v; v = v->next)
		{
			s += v->position;
		}
		s /= float32(primary.GetVertexList().count);

		primary.Translate(-s);

		// Build the dual hull 
		u32 dvCount = 0;
		qhFace** dfs = (qhFace**)b3Alloc(primary.GetFaceList().count * sizeof(qhFace*));
		b3Vec3* dvs = (b3Vec3*)b3Alloc(primary.GetFaceList().count * sizeof(b3Vec3));

		for (qhFace* f = primary.GetFaceList().head; f; f = f->next)
		{
			b3Plane plane = f->plane;
			B3_ASSERT(plane.offset > 0.0f);
			b3Vec3 v = plane.normal / plane.offset;
			b3Vec3 vn = plane.normal;

			bool unique = true;

			for (u32 j = 0; j < dvCount; ++j)
			{
				qhFace*& df = dfs[j];
				b3Vec3& dv = dvs[j];
				b3Vec3 dvn = b3Normalize(dv);

				// ~45 degrees
				const float32 kTol = 0.7f;

				if (b3Dot(vn, dvn) > kTol)
				{
					if (f->area > df->area)
					{
						df = f;
						dv = v;
					}

					unique = false;
					break;
				}
			}

			if (unique)
			{
				dfs[dvCount] = f;
				dvs[dvCount] = v;
				++dvCount;
			}
		}

		b3Free(dfs);

		if (dvCount < 4)
		{
			b3Free(dvs);
			return;
		}

		qhHull dual;
		dual.Construct(dvs, dvCount);
		b3Free(dvs);

		// Recover the simplified hull in primary space. 
		u32 pvCount = 0;
		b3Vec3* pvs = (b3Vec3*)b3Alloc(dual.GetFaceList().count * sizeof(b3Vec3));
		for (qhFace* f = dual.GetFaceList().head; f; f = f->next)
		{
			b3Plane plane = f->plane;
			B3_ASSERT(plane.offset > 0.0f);
			b3Vec3 v = plane.normal / plane.offset;

			bool unique = true;

			for (u32 j = 0; j < pvCount; ++j)
			{
				if (b3DistanceSquared(v, pvs[j]) <= B3_LINEAR_SLOP * B3_LINEAR_SLOP)
				{
					unique = false;
					break;
				}
			}

			if (unique)
			{
				pvs[pvCount++] = v;
			}
		}

		if (pvCount < 4)
		{
			b3Free(pvs);
			return;
		}

		hull.Construct(pvs, pvCount);
		b3Free(pvs);

		// Translate the hull back to the origin
		hull.Translate(s);
	}
	else
	{
		hull.Construct(vs0, vs0Count);
		b3Free(vs0);
	}

	// Convert the constructed hull into a run-time hull.
	b3UniqueStackArray<qhVertex*> vs;
	b3UniqueStackArray<qhHalfEdge*> es;

	// Add vertices to the map
	for (qhVertex* vertex = hull.GetVertexList().head; vertex != NULL; vertex = vertex->next)
	{
		vs.PushBack(vertex);
	}

	// Add half-edges to the map
	for (qhFace* face = hull.GetFaceList().head; face != NULL; face = face->next)
	{
		// Add half-edges 
		qhHalfEdge* begin = face->edge;
		qhHalfEdge* edge = begin;
		do
		{
			// Add half-edge
			u32 iedge = es.PushBack(edge);

			// Add half-edge just after its twin
			u32 itwin = es.PushBack(edge->twin);

			edge = edge->next;
		} while (edge != begin);
	}

	hullVertices.Resize(hull.GetVertexList().count);
	hullEdges.Resize(es.elements.Count());
	hullFaces.Resize(hull.GetFaceList().count);
	hullPlanes.Resize(hull.GetFaceList().count);

	// Build and link the features
	u32 iface = 0;
	for (qhFace* face = hull.GetFaceList().head; face != NULL; face = face->next)
	{
		// Build and link the half-edges 
		b3Face* hface = hullFaces.Get(iface);

		hullPlanes[iface] = face->plane;

		qhHalfEdge* begin = face->edge;
		hface->edge = es.PushBack(begin);

		qhHalfEdge* edge = begin;
		do
		{
			qhVertex* v = edge->tail;
			u32 iv = vs.PushBack(v);
			hullVertices[iv] = v->position;

			u32 iedge = es.PushBack(edge);
			b3HalfEdge* hedge = hullEdges.Get(iedge);
			hedge->face = iface;
			hedge->origin = iv;

			qhHalfEdge* twin = edge->twin;
			u32 itwin = es.PushBack(twin);
			b3HalfEdge* htwin = hullEdges.Get(itwin);
			htwin->twin = iedge;

			hedge->twin = itwin;

			qhHalfEdge* next = edge->next;
			u32 inext = es.PushBack(next);

			hullEdges[iedge].next = inext;

			edge = next;
		} while (edge != begin);

		++iface;
	}

	vertexCount = hullVertices.Count();
	edgeCount = hullEdges.Count();
	faceCount = hullFaces.Count();

	// Validate
	Validate();

	// Compute the centroid.
	centroid = b3ComputeCentroid(this);
}

void b3QHull::SetAsSphere(float32 radius)
{
	enum
	{
		e_rings = 8,
		e_sectors = 8,
		e_vertexCount = e_rings * e_sectors
	};

	float32 R = 1.0f / float32(e_rings - 1);
	float32 S = 1.0f / float32(e_sectors - 1);

	b3Vec3 vs[e_vertexCount];

	u32 vc = 0;
	for (u32 r = 0; r < e_rings; r++)
	{
		for (u32 s = 0; s < e_sectors; s++)
		{
			float32 y = sin(-0.5f * B3_PI + B3_PI * r * R);
			float32 x = cos(2.0f * B3_PI * s * S) * sin(B3_PI * r * R);
			float32 z = sin(2.0f * B3_PI * s * S) * sin(B3_PI * r * R);

			vs[vc].Set(x, y, z);
			vs[vc] *= radius;
			++vc;
		}
	}

	// Set
	Set(sizeof(b3Vec3), vs, e_vertexCount, false);
}

void b3QHull::SetAsCylinder(float32 radius, float32 ey)
{
	B3_ASSERT(radius > 0.0f);
	B3_ASSERT(ey > 0.0f);

	const u32 kEdgeCount = 20;
	const u32 kVertexCount = 4 * kEdgeCount;
	b3Vec3 vs[kVertexCount];

	u32 count = 0;

	float32 kAngleInc = 2.0f * B3_PI / float32(kEdgeCount);
	b3Quat q = b3QuatRotationY(kAngleInc);

	{
		b3Vec3 center(0.0f, -ey, 0.0f);
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
		b3Vec3 center(0.0f, ey, 0.0f);
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
	Set(sizeof(b3Vec3), vs, count, false);
}

void b3QHull::SetAsCone(float32 radius, float32 ey)
{
	B3_ASSERT(radius > 0.0f);
	B3_ASSERT(ey > 0.0f);

	const u32 kEdgeCount = 20;
	const u32 kVertexCount = 2 * kEdgeCount + 1;
	b3Vec3 vs[kVertexCount];

	u32 count = 0;

	float32 kAngleInc = 2.0f * B3_PI / float32(kEdgeCount);
	b3Quat q = b3QuatRotationY(kAngleInc);

	b3Vec3 center(0.0f, -ey, 0.0f);
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

	vs[count++].Set(0.0f, ey, 0.0f);

	// Set
	Set(sizeof(b3Vec3), vs, count, false);
}