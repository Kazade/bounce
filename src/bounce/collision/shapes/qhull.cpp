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

// Used to map pointers to indices 
// If more performance is required then a use hash-map
template<class T, u32 N>
struct b3UniqueArray
{
	b3UniqueArray()
	{
		count = 0;
	}
	
	// Add the value if not found
	void Add(const T& value)
	{
		B3_ASSERT(count < N);
		values[count++] = value;
	}

	// Add the value if not found
	// Return value index if added
	// Return N if the array is full
	u32 Find(const T& value)
	{
		for (u32 i = 0; i < count; ++i)
		{
			if (values[i] == value)
			{
				return i;
			}
		}

		if (count == N)
		{
			return N;
		}

		values[count++] = value;
		return count - 1;
	}
	
	T values[N];
	u32 count;
};

//
static b3Vec3 b3ComputeCentroid(b3QHull* hull)
{
	B3_ASSERT(hull->vertexCount >= 4);

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
	B3_ASSERT(count >= 4 && count <= B3_MAX_HULL_VERTICES);

	// Clamp vertices into range [0, B3_MAX_HULL_VERTICES]
	u32 n = b3Min(count, u32(B3_MAX_HULL_VERTICES));

	// Copy points into local buffer, remove coincident points.
	b3Vec3 ps[B3_MAX_HULL_VERTICES];
	u32 psCount = 0;
	for (u32 i = 0; i < n; ++i)
	{
		b3Vec3 p = points[i];

		bool unique = true;

		for (u32 j = 0; j < psCount; ++j)
		{
			if (b3DistanceSquared(p, ps[j]) <= B3_LINEAR_SLOP * B3_LINEAR_SLOP)
			{
				unique = false;
				break;
			}
		}

		if (unique)
		{
			ps[psCount++] = p;
		}
	}

	if (psCount < 4)
	{
		// Polyhedron is degenerate.
		return;
	}

	// Create a convex hull.
	
	// Allocate memory buffer for the worst case.
	u32 qhBufferSize = qhGetBufferSize(B3_MAX_HULL_VERTICES);
	void* qhBuffer = b3Alloc(qhBufferSize);

	// Build
	qhHull hull;
	hull.Construct(qhBuffer, ps, psCount);

	if (hull.GetVertexList().count > B3_MAX_HULL_VERTICES)
	{
		// Vertex excess
		b3Free(qhBuffer);
		return;
	}

	if (hull.GetFaceList().count > B3_MAX_HULL_FACES)
	{
		// Face excess
		b3Free(qhBuffer);
		return;
	}
	
	// Convert the constructed hull into a run-time hull.
	b3UniqueArray<qhVertex*, B3_MAX_HULL_VERTICES> vs;
	b3UniqueArray<qhHalfEdge*, B3_MAX_HULL_EDGES> es;
	u32 fs_count = 0;
	
	// Add vertices to the map
	for (qhVertex* vertex = hull.GetVertexList().head; vertex != NULL; vertex = vertex->next)
	{
		// Add vertex
		vs.Add(vertex);
	}

	// Add faces and to the map
	for (qhFace* face = hull.GetFaceList().head; face != NULL; face = face->next)
	{
		// Add face
		B3_ASSERT(fs_count < B3_MAX_HULL_FACES);
		++fs_count;

		// Add vertices and half-edges 
		qhHalfEdge* begin = face->edge;
		qhHalfEdge* edge = begin;
		do
		{
			// Add half-edge
			u32 iedge = es.Find(edge);
			if (iedge == B3_MAX_HULL_EDGES)
			{
				// Half-edge excess
				b3Free(qhBuffer);
				return;
			}

			// Add half-edge just after its twin
			u32 itwin = es.Find(edge->twin);
			if (itwin == B3_MAX_HULL_EDGES)
			{
				// Half-edge excess
				b3Free(qhBuffer);
				return;
			}

			edge = edge->next;
		} while (edge != begin);
	}
	
	// Build and link the features
	u32 iface = 0;
	for (qhFace* face = hull.GetFaceList().head; face != NULL; face = face->next)
	{
		// Build and link the half-edges 
		b3Face* hface = faces + iface;
		
		planes[iface] = face->plane;

		qhHalfEdge* begin = face->edge;
		hface->edge = (u8)es.Find(begin);

		qhHalfEdge* edge = begin;
		do
		{
			qhVertex* v = edge->tail;
			u8 iv = (u8)vs.Find(v);
			vertices[iv] = v->position;

			u8 iedge = (u8)es.Find(edge);
			b3HalfEdge* hedge = edges + iedge;
			hedge->face = u8(iface);
			hedge->origin = iv;

			qhHalfEdge* twin = edge->twin;
			u8 itwin = (u8)es.Find(twin);
			b3HalfEdge* htwin = edges + itwin;
			htwin->twin = iedge;

			hedge->twin = itwin;

			qhHalfEdge* next = edge->next;
			u8 inext = (u8)es.Find(next);

			edges[iedge].next = inext;

			edge = next;
		} while (edge != begin);

		++iface;
	}

	b3Free(qhBuffer);

	B3_ASSERT(vs.count <= B3_MAX_HULL_VERTICES);
	vertexCount = vs.count;

	B3_ASSERT(es.count <= B3_MAX_HULL_EDGES);
	edgeCount = es.count;
	
	B3_ASSERT(fs_count <= B3_MAX_HULL_FACES);
	faceCount = fs_count;

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