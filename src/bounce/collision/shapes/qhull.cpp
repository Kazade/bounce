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
template<class T, u32 N>
struct b3UniqueArray
{
	b3UniqueArray()
	{
		count = 0;
	}

	void PushBack(const T& value)
	{
		// B3_ASSERT(Find(value) == count)
		B3_ASSERT(count < N);
		values[count++] = value;
	}

	u32 Find(const T& value) const
	{
		for (u32 i = 0; i < count; ++i)
		{
			if (values[i] == value)
			{
				return i;
			}
		}
		return count;
	}
	
	T values[N];
	u32 count;
};

//
template<class T> 
static inline u32 b3Find(const T* values, u32 count, const T* value)
{
	for (u32 i = 0; i < count; ++i)
	{
		if (values[i] == value)
		{
			return i;
		}
	}
	B3_ASSERT(false);
}

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
			const float32 kTol = 0.5f * B3_LINEAR_SLOP;
			if (b3DistanceSquared(p, ps[j]) < kTol * kTol)
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
	const u32 qhBufferSize = qhGetBufferSize(B3_MAX_HULL_VERTICES);
	u8 qhBuffer[qhBufferSize];

	// Build
	qhHull hull;
	hull.Construct(qhBuffer, ps, psCount);

	// Cheaply convert the constructed hull into a run-time hull.

	// Unique vertices and edges
	b3UniqueArray<qhVertex*, B3_MAX_HULL_VERTICES> vs;
	b3UniqueArray<qhHalfEdge*, B3_MAX_HULL_EDGES> es;
	u32 fs_count = 0;

	// Face half-edges
	u8 fhs[B3_MAX_HULL_EDGES];
	u32 nfh = 0;

	const qhList<qhFace>& faceList = hull.GetFaceList();
	qhFace* face = faceList.head;
	while (face)
	{
		if (fs_count == B3_MAX_HULL_FACES)
		{
			// Face excess
			return;
		}

		// Add face
		B3_ASSERT(fs_count < B3_MAX_HULL_FACES);
		b3Face* f = faces + fs_count;
		u32 fi = fs_count;
		++fs_count;

		planes[fi] = face->plane;
		
		qhHalfEdge* begin = face->edge;
		qhHalfEdge* edge = begin;
		do
		{
			qhHalfEdge* twin = edge->twin;
			
			qhVertex* v1 = edge->tail;
			qhVertex* v2 = twin->tail;

			u32 iv1 = vs.Find(v1);
			if (iv1 == vs.count)
			{
				// Vertex excess
				if (vs.count == B3_MAX_HULL_VERTICES)
				{
					return;
				}

				// Add vertex
				vs.PushBack(v1);
			}

			u32 iv2 = vs.Find(v2);
			if (iv2 == vs.count)
			{
				// Vertex excess
				if (vs.count == B3_MAX_HULL_VERTICES)
				{
					return;
				}
				
				// Add vertex
				vs.PushBack(v2);
			}
			
			u32 ie2 = es.Find(edge);
			if(ie2 == es.count)
			{
				// Edge excess
				if (es.count + 2 >= B3_MAX_HULL_EDGES)
				{
					return;
				}

				// Add half-edges
				u32 ie1 = es.count;
				es.PushBack(edge);

				u32 ie2 = es.count;
				es.PushBack(twin);

				// Link half-edges
				b3HalfEdge* e1 = edges + ie1;
				e1->face = u8(fi);
				e1->origin = iv1;
				e1->twin = ie2;

				b3HalfEdge* e2 = edges + ie2;
				e2->face = B3_NULL_HULL_FEATURE;
				e2->origin = iv2;
				e2->twin = ie1;

				B3_ASSERT(nfh < B3_MAX_HULL_EDGES);
				fhs[nfh++] = ie1;
			}
			else
			{
				b3HalfEdge* e2 = edges + ie2;
				
				B3_ASSERT(e2->face == B3_NULL_HULL_FEATURE);
				e2->face = u8(fi);

				B3_ASSERT(nfh < B3_MAX_HULL_EDGES);
				fhs[nfh++] = ie2;
			}

			edge = edge->next;
		} while (edge != begin);
		
		// Link any face half-edge to face
		B3_ASSERT(nfh > 0);
		f->edge = fhs[0];
		
		// Link half-edge list 
		for (u32 i = 0; i < nfh; ++i)
		{
			u8 edge = fhs[i];
			u8 nextEdge = i < nfh - 1 ? i + 1 : 0;
			
			edges[edge].next = fhs[nextEdge];
		}

		nfh = 0;

		face = face->next;
	}

	B3_ASSERT(vs.count <= B3_MAX_HULL_VERTICES);
	vertexCount = vs.count;
	for (u32 i = 0; i < vs.count; ++i)
	{
		vertices[i] = vs.values[i]->position;
	}

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