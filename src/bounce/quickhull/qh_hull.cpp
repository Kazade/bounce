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

#include <bounce/quickhull/qh_hull.h>
#include <bounce/common/draw.h>

static float32 qhFindAABB(u32 iMin[3], u32 iMax[3], const b3Vec3* vs, u32 count)
{
	b3Vec3 min(B3_MAX_FLOAT, B3_MAX_FLOAT, B3_MAX_FLOAT);
	iMin[0] = 0;
	iMin[1] = 0;
	iMin[2] = 0;

	b3Vec3 max(-B3_MAX_FLOAT, -B3_MAX_FLOAT, -B3_MAX_FLOAT);
	iMax[0] = 0;
	iMax[1] = 0;
	iMax[2] = 0;

	for (u32 i = 0; i < count; ++i)
	{
		b3Vec3 v = vs[i];

		for (u32 j = 0; j < 3; ++j)
		{
			if (v[j] < min[j])
			{
				min[j] = v[j];
				iMin[j] = i;
			}

			if (v[j] > max[j])
			{
				max[j] = v[j];
				iMax[j] = i;
			}
		}
	}

	return 3.0f * (b3Abs(max.x) + b3Abs(max.y) + b3Abs(max.z)) * B3_EPSILON;
}

qhHull::qhHull()
{
	m_vertexList.head = NULL;
	m_vertexList.count = 0;

	m_faceList.head = NULL;
	m_faceList.count = 0;

	m_buffer = NULL;

	m_vertexCapacity = 0;
	m_vertexCount = 0;

	m_edgeCapacity = 0;
	m_edgeCount = 0;

	m_faceCapacity = 0;
	m_faceCount = 0;
}

qhHull::~qhHull()
{
	qhVertex* v = m_vertexList.head;
	while (v)
	{
		qhVertex* v0 = v;
		v = v->next;

		FreeVertex(v0);
	}

	B3_ASSERT(m_vertexCount == 0);

	qhFace* f = m_faceList.head;
	while (f)
	{
		qhFace* f0 = f;
		f = f->next;

		qhHalfEdge* e = f0->edge;
		do
		{
			qhHalfEdge* e0 = e;
			e = e->next;

			FreeEdge(e0);
		} while (e != f0->edge);

		FreeFace(f0);
	}

	B3_ASSERT(m_edgeCount == 0);
	B3_ASSERT(m_faceCount == 0);

	b3Free(m_buffer);
}

void qhHull::Construct(const b3Vec3* vs, u32 count)
{
	B3_ASSERT(m_buffer == NULL);
	B3_ASSERT(count > 0 && count >= 4);

	// Compute memory buffer size for the worst case.
	u32 size = 0;

	// Hull using Euler's Formula
	u32 V = count;
	u32 E = 3 * V - 6;
	u32 HE = 2 * E;
	u32 F = 2 * V - 4;

	size += V * sizeof(qhVertex);
	size += HE * sizeof(qhHalfEdge);
	size += F * sizeof(qhFace);

	// Horizon 
	size += HE * sizeof(qhHalfEdge*);

	// Saved horizon vertices
	// One vertex per horizon edge
	size += HE * sizeof(qhVertex*);

	// Saved conflict vertices
	size += V * sizeof(qhVertex*);

	// New Faces
	// One face per horizon edge
	size += HE * sizeof(qhFace*);

	// Allocate memory buffer 
	m_buffer = b3Alloc(size);

	// Initialize free lists
	m_vertexCapacity = V;
	m_vertexCount = m_vertexCapacity;
	m_freeVertices = NULL;
	qhVertex* vertices = (qhVertex*)m_buffer;
	for (u32 i = 0; i < V; ++i)
	{
		vertices[i].active = true;
		FreeVertex(vertices + i);
	}
	B3_ASSERT(m_vertexCount == 0);

	m_edgeCapacity = HE;
	m_edgeCount = m_edgeCapacity;
	m_freeEdges = NULL;
	qhHalfEdge* edges = (qhHalfEdge*)((u8*)vertices + V * sizeof(qhVertex));
	for (u32 i = 0; i < HE; ++i)
	{
		edges[i].active = true;
		FreeEdge(edges + i);
	}
	B3_ASSERT(m_edgeCount == 0);

	m_faceCapacity = F;
	m_faceCount = m_faceCapacity;
	m_freeFaces = NULL;
	qhFace* faces = (qhFace*)((u8*)edges + HE * sizeof(qhHalfEdge));
	for (u32 i = 0; i < F; ++i)
	{
		qhFace* f = faces + i;
		f->conflictList.head = NULL;
		f->conflictList.count = 0;
		f->active = true;
		FreeFace(f);
	}
	B3_ASSERT(m_faceCount == 0);

	m_horizon = (qhHalfEdge**)((u8*)faces + F * sizeof(qhFace));
	m_horizonCount = 0;

	m_horizonVertices = (qhVertex**)((u8*)m_horizon + HE * sizeof(qhHalfEdge*));

	m_conflictVertices = (qhVertex**)((u8*)m_horizonVertices + HE * sizeof(qhVertex*));
	m_conflictCount = 0;

	m_newFaces = (qhFace**)((u8*)m_conflictVertices + V * sizeof(qhVertex*));
	m_newFaceCount = 0;

	m_iterations = 0;

	// Build initial tetrahedron
	if (!BuildInitialHull(vs, count))
	{
		return;
	}

	// Run Quickhull
	qhVertex* eye = FindEyeVertex();
	while (eye)
	{
		Validate();
		ValidateConvexity();

		AddEyeVertex(eye);

		eye = FindEyeVertex();

		++m_iterations;
	}

	Validate();
	ValidateConvexity();

#if 0
	// Ensure the hull contains the original vertex set
	for (u32 i = 0; i < count; ++i)
	{
		b3Vec3 v = vs[i];

		for (qhFace* f = m_faceList.head; f; f = f->next)
		{
			float32 d = b3Distance(v, f->plane);
			B3_ASSERT(d < m_tolerance);
		}
	}
#endif
}

bool qhHull::BuildInitialHull(const b3Vec3* vertices, u32 vertexCount)
{
	if (vertexCount < 4)
	{
		B3_ASSERT(false);
		return false;
	}

	u32 i1 = 0, i2 = 0;

	{
		// Find the points that maximizes the distance along the 
		// canonical axes.
		// Also store a tolerance for coplanarity checks.
		u32 aabbMin[3], aabbMax[3];
		m_tolerance = qhFindAABB(aabbMin, aabbMax, vertices, vertexCount);
		
		// Find the longest segment.
		float32 d0 = 0.0f;

		for (u32 i = 0; i < 3; ++i)
		{
			b3Vec3 A = vertices[aabbMin[i]];
			b3Vec3 B = vertices[aabbMax[i]];

			float32 d = b3DistanceSquared(A, B);

			if (d > d0)
			{
				d0 = d;
				i1 = aabbMin[i];
				i2 = aabbMax[i];
			}
		}

		// Coincidence check
		if (d0 <= B3_EPSILON * B3_EPSILON)
		{
			B3_ASSERT(false);
			return false;
		}
	}

	B3_ASSERT(i1 != i2);

	b3Vec3 A = vertices[i1];
	b3Vec3 B = vertices[i2];

	u32 i3 = 0;

	{
		// Find the triangle which has the largest area.
		float32 a0 = 0.0f;

		for (u32 i = 0; i < vertexCount; ++i)
		{
			if (i == i1 || i == i2)
			{
				continue;
			}

			b3Vec3 C = vertices[i];

			float32 a = b3AreaSquared(A, B, C);

			if (a > a0)
			{
				a0 = a;
				i3 = i;
			}
		}

		// Colinear check.
		if (a0 <= (2.0f * B3_EPSILON) * (2.0f * B3_EPSILON))
		{
			B3_ASSERT(false);
			return false;
		}
	}

	B3_ASSERT(i3 != i1 && i3 != i2);

	b3Vec3 C = vertices[i3];

	b3Vec3 N = b3Cross(B - A, C - A);
	N.Normalize();

	b3Plane plane(N, A);

	u32 i4 = 0;

	{
		// Find the furthest point from the triangle plane.
		float32 d0 = 0.0f;

		for (u32 i = 0; i < vertexCount; ++i)
		{
			if (i == i1 || i == i2 || i == i3)
			{
				continue;
			}

			b3Vec3 D = vertices[i];

			float32 d = b3Abs(b3Distance(D, plane));

			if (d > d0)
			{
				d0 = d;
				i4 = i;
			}
		}

		// Coplanar check.
		if (d0 <= m_tolerance)
		{
			B3_ASSERT(false);
			return false;
		}
	}

	B3_ASSERT(i4 != i1 && i4 != i2 && i4 != i3);

	// Add okay simplex to the hull.
	b3Vec3 D = vertices[i4];

	qhVertex* v1 = AddVertex(A);
	qhVertex* v2 = AddVertex(B);
	qhVertex* v3 = AddVertex(C);
	qhVertex* v4 = AddVertex(D);

	if (b3Distance(D, plane) < 0.0f)
	{
		AddFace(v1, v2, v3);
		AddFace(v4, v2, v1);
		AddFace(v4, v3, v2);
		AddFace(v4, v1, v3);
	}
	else
	{
		// Ensure CCW order.
		AddFace(v1, v3, v2);
		AddFace(v4, v1, v2);
		AddFace(v4, v2, v3);
		AddFace(v4, v3, v1);
	}

	// Connectivity check.
	Validate();

	// Add remaining points to the conflict lists on each face.
	for (u32 i = 0; i < vertexCount; ++i)
	{
		// Skip hull vertices.
		if (i == i1 || i == i2 || i == i3 || i == i4)
		{
			continue;
		}

		b3Vec3 p = vertices[i];

		// Ignore internal points since they can't be in the hull.
		float32 d0 = m_tolerance;
		qhFace* f0 = NULL;

		for (qhFace* f = m_faceList.head; f != NULL; f = f->next)
		{
			float32 d = b3Distance(p, f->plane);
			if (d > d0)
			{
				d0 = d;
				f0 = f;
			}
		}

		if (f0)
		{
			qhVertex* v = AllocateVertex();
			v->position = p;
			v->conflictFace = f0;
			f0->conflictList.PushFront(v);
		}
	}

	return true;
}

qhVertex* qhHull::FindEyeVertex() const
{
	// Find the furthest conflict point.
	float32 d0 = m_tolerance;
	qhVertex* v0 = NULL;

	for (qhFace* f = m_faceList.head; f != NULL; f = f->next)
	{
		for (qhVertex* v = f->conflictList.head; v != NULL; v = v->next)
		{
			float32 d = b3Distance(v->position, f->plane);
			if (d > d0)
			{
				d0 = d;
				v0 = v;
			}
		}
	}

	return v0;
}

void qhHull::AddEyeVertex(qhVertex* eye)
{
	FindHorizon(eye);
	AddNewFaces(eye);
	MergeNewFaces();
	ResolveOrphans();
}

void qhHull::FindHorizon(qhVertex* eye)
{
	// Mark faces
	for (qhFace* face = m_faceList.head; face != NULL; face = face->next)
	{
		float32 d = b3Distance(eye->position, face->plane);
		if (d > m_tolerance)
		{
			face->mark = qhFaceMark::e_visible;
		}
		else
		{
			face->mark = qhFaceMark::e_invisible;
		}
	}

	// Find the horizon 
	m_horizonCount = 0;
	for (qhFace* face = m_faceList.head; face != NULL; face = face->next)
	{
		if (face->mark == qhFaceMark::e_invisible)
		{
			continue;
		}

		qhHalfEdge* begin = face->edge;
		qhHalfEdge* edge = begin;
		do
		{
			qhHalfEdge* twin = edge->twin;
			qhFace* other = twin->face;

			if (other->mark == qhFaceMark::e_invisible)
			{
				m_horizon[m_horizonCount++] = edge;
			}

			edge = edge->next;
		} while (edge != begin);
	}

	// Sort the horizon in CCW order 
	B3_ASSERT(m_horizonCount > 0);
	for (u32 i = 0; i < m_horizonCount - 1; ++i)
	{
		qhHalfEdge* e1 = m_horizon[i]->twin;
		qhVertex* v1 = e1->tail;

		for (u32 j = i + 1; j < m_horizonCount; ++j)
		{
			// Ensure unique edges
			B3_ASSERT(m_horizon[i] != m_horizon[j]);

			qhHalfEdge* e2 = m_horizon[j];
			qhVertex* v2 = e2->tail;

			if (v1 == v2)
			{
				b3Swap(m_horizon[j], m_horizon[i + 1]);
				break;
			}
		}
	}
}

void qhHull::AddNewFaces(qhVertex* eye)
{
	// Ensure CCW horizon order. 
	// Usually it can fail hit if face merging is disable.
	B3_ASSERT(m_horizonCount > 0);
	for (u32 i = 0; i < m_horizonCount; ++i)
	{
		qhHalfEdge* e1 = m_horizon[i]->twin;

		u32 j = i + 1 < m_horizonCount ? i + 1 : 0;
		qhHalfEdge* e2 = m_horizon[j];

		B3_ASSERT(e1->tail == e2->tail);
	}

	// Save horizon vertices
	for (u32 i = 0; i < m_horizonCount; ++i)
	{
		qhHalfEdge* edge = m_horizon[i];

		m_horizonVertices[i] = edge->tail;
	}

	// Remove the eye vertex from the conflict list
	b3Vec3 eyePosition = eye->position;

	eye->conflictFace->conflictList.Remove(eye);
	FreeVertex(eye);

	// Add the eye point to the hull
	qhVertex* v1 = AddVertex(eyePosition);

	// Save conflict vertices
	m_conflictCount = 0;

	// Remove visible faces
	qhFace* f = m_faceList.head;
	while (f)
	{
		// Skip invisible faces.
		if (f->mark == qhFaceMark::e_invisible)
		{
			f = f->next;
			continue;
		}

		qhVertex* v = f->conflictList.head;
		while (v)
		{
			// Save vertex
			m_conflictVertices[m_conflictCount++] = v;

			// Remove vertex from face
			v->conflictFace = NULL;
			v = f->conflictList.Remove(v);
		}

		// Remove face
		f = RemoveFace(f);
	}

	// Add new faces to the hull
	m_newFaceCount = 0;
	for (u32 i = 0; i < m_horizonCount; ++i)
	{
		u32 j = i + 1 < m_horizonCount ? i + 1 : 0;

		qhVertex* v2 = m_horizonVertices[i];
		qhVertex* v3 = m_horizonVertices[j];

		m_newFaces[m_newFaceCount++] = AddFace(v1, v2, v3);
	}
}

void qhHull::ResolveOrphans()
{
	// Move the orphaned conflict vertices into the new faces
	// Remove internal conflict vertices
	for (u32 i = 0; i < m_conflictCount; ++i)
	{
		qhVertex* v = m_conflictVertices[i];

		b3Vec3 p = v->position;

		float32 d0 = m_tolerance;
		qhFace* f0 = NULL;

		for (u32 j = 0; j < m_newFaceCount; ++j)
		{
			qhFace* nf = m_newFaces[j];

			// Was the face deleted due to merging?
			if (nf->active == false)
			{
				continue;
			}

			float32 d = b3Distance(p, nf->plane);
			if (d > d0)
			{
				d0 = d;
				f0 = nf;
			}
		}

		if (f0)
		{
			// Add conflict vertex to the new face
			f0->conflictList.PushFront(v);
			v->conflictFace = f0;
		}
		else
		{
			// Remove conflict vertex
			FreeVertex(v);
		}
	}
}

qhVertex* qhHull::AddVertex(const b3Vec3& position)
{
	qhVertex* v = AllocateVertex();
	v->position = position;
	v->conflictFace = NULL;

	m_vertexList.PushFront(v);

	return v;
}

qhHalfEdge* qhHull::FindHalfEdge(const qhVertex* v1, const qhVertex* v2) const
{
	for (qhFace* face = m_faceList.head; face != NULL; face = face->next)
	{
		qhHalfEdge* e = face->edge;
		do
		{
			B3_ASSERT(e->active == true);
			B3_ASSERT(e->twin != NULL);
			B3_ASSERT(e->twin->active == true);

			if (e->tail == v1 && e->twin->tail == v2)
			{
				return e;
			}

			if (e->tail == v2 && e->twin->tail == v1)
			{
				return e->twin;
			}

			e = e->next;
		} while (e != face->edge);
	}
	return NULL;
}

static B3_FORCE_INLINE b3Vec3 b3Newell(const b3Vec3& a, const b3Vec3& b)
{
	return b3Vec3((a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y));
}

// Compute face centroid, normal, and area
static void b3ResetFaceData(qhFace* face)
{
	// Compute polygon centroid
	b3Vec3 c;
	c.SetZero();

	u32 count = 0;
	qhHalfEdge* e = face->edge;
	do
	{
		b3Vec3 v = e->tail->position;
		c += v;
		++count;
		e = e->next;
	} while (e != face->edge);

	B3_ASSERT(count >= 3);
	c /= float32(count);

	// Compute normal  
	b3Vec3 n;
	n.SetZero();

	e = face->edge;
	do
	{
		b3Vec3 v1 = e->tail->position;
		b3Vec3 v2 = e->next->tail->position;

		// Shift the polygon origin to the centroid
		v1 -= c;
		v2 -= c;

		// Apply Newell's method
		n += b3Newell(v1, v2);

		e = e->next;
	} while (e != face->edge);

	// Centroid
	face->center = c;

	float32 len = b3Length(n);
	B3_ASSERT(len > B3_EPSILON);
	n /= len;

	// Area
	face->area = 0.5f * len;

	// Normal
	face->plane.normal = n;
	face->plane.offset = b3Dot(n, c);
}

static u32 b3VertexCount(const qhFace* face)
{
	u32 n = 0;
	qhHalfEdge* e = face->edge;
	do
	{
		++n;
		e = e->next;
	} while (e != face->edge);
	return n;
}

qhHalfEdge* qhHull::FixMerge(qhFace* face1, qhHalfEdge* ein)
{
	qhHalfEdge* eout = ein->next;

	B3_ASSERT(ein->twin->face == eout->twin->face);

	qhFace* face3 = ein->twin->face;

	u32 count = b3VertexCount(face3);

	// Is the face 3 a triangle?
	if (count == 3)
	{
		qhHalfEdge* nextEdge = eout->next;

		// Unlink incoming edge from face 1
		B3_ASSERT(ein->prev->next == ein);
		ein->prev->next = ein->twin->next;

		// Unlink incoming edge twin from face 3
		B3_ASSERT(ein->twin->next->prev == ein->twin);
		ein->twin->next->prev = ein->prev;

		B3_ASSERT(ein->face == face1);
		if (face1->edge == ein)
		{
			face1->edge = ein->prev;
		}

		// Set incoming edge twin face reference the face 1
		B3_ASSERT(ein->twin->next->face == face3);
		ein->twin->next->face = face1;

		// Unlink outgoing edge from face 1
		B3_ASSERT(eout->next->prev == eout);
		eout->next->prev = eout->twin->prev;
		B3_ASSERT(eout->twin->prev->next == eout->twin);
		eout->twin->prev->next = eout->next;

		B3_ASSERT(eout->face == face1);
		if (face1->edge == eout)
		{
			face1->edge = eout->next;
		}

		// Reset face 1 data
		b3ResetFaceData(face1);

		// Validate face 1
		Validate(face1);

		// Remove outgoing vertex
		m_vertexList.Remove(eout->tail);
		FreeVertex(eout->tail);

		// Remove incoming edge
		FreeEdge(ein->twin);
		FreeEdge(ein);

		// Remove outgoing edge
		FreeEdge(eout->twin);
		FreeEdge(eout);

		// Move face 3 conflict vertices into face 1
		qhVertex* v = face3->conflictList.head;
		while (v)
		{
			qhVertex* v0 = v;
			v = face3->conflictList.Remove(v);

			face1->conflictList.PushFront(v0);
			v0->conflictFace = face1;
		}

		// Remove face 3
		m_faceList.Remove(face3);
		FreeFace(face3);

		// Return the next edge
		return nextEdge;
	}
	else
	{
		qhHalfEdge* nextEdge = eout->next;

		// Extend the incoming edge to the next vertex
		B3_ASSERT(ein->twin->tail == eout->tail);
		ein->twin->tail = eout->twin->tail;

		// Remove outgoing vertex
		m_vertexList.Remove(eout->tail);
		FreeVertex(eout->tail);

		// Unlink outgoing edge from face 1
		B3_ASSERT(eout->prev->next == eout);
		eout->prev->next = eout->next;
		B3_ASSERT(eout->next->prev == eout);
		eout->next->prev = eout->prev;

		B3_ASSERT(eout->face == face1);
		if (face1->edge == eout)
		{
			face1->edge = eout->next;
		}

		// Reset face 1 data
		b3ResetFaceData(face1);

		// Validate face 1
		Validate(face1);

		// Unlink outgoing edge twin from face 3
		B3_ASSERT(eout->twin->prev->next == eout->twin);
		eout->twin->prev->next = eout->twin->next;
		B3_ASSERT(eout->twin->next->prev == eout->twin);
		eout->twin->next->prev = eout->twin->prev;

		B3_ASSERT(eout->twin->face == face3);
		if (face3->edge == eout->twin)
		{
			face3->edge = eout->twin->next;
		}

		// Remove outgoing edge
		FreeEdge(eout->twin);
		FreeEdge(eout);

		// Return the next edge
		return nextEdge;
	}
	
	// Return the next edge to the given edge
	return eout;
}

qhFace* qhHull::RemoveEdge(qhHalfEdge* edge)
{
	B3_ASSERT(edge->active == true);

	qhFace* face1 = edge->face;
	B3_ASSERT(face1->active == true);
	B3_ASSERT(edge->twin->active == true);
	qhFace* face2 = edge->twin->face;

	// Edge must be shared.
	B3_ASSERT(face2 != NULL);
	B3_ASSERT(face2->active == true);
	B3_ASSERT(face2 != face1);

	// Merge face 2 into face 1

	// Set face 2 edges owner face to face 1,
	// except the twin edge which will be deleted
	for (qhHalfEdge* e2 = edge->twin->next; e2 != edge->twin; e2 = e2->next)
	{
		B3_ASSERT(e2->face == face2);
		e2->face = face1;
	}

	// Set the face 1 to reference a non-deleted edge
	B3_ASSERT(edge->face == face1);
	if (face1->edge == edge)
	{
		face1->edge = edge->next;
	}

	// Unlink edge from face 1
	B3_ASSERT(edge->prev->next == edge);
	edge->prev->next = edge->twin->next;
	B3_ASSERT(edge->next->prev == edge);
	edge->next->prev = edge->twin->prev;

	B3_ASSERT(edge->twin->prev->next == edge->twin);
	edge->twin->prev->next = edge->next;
	B3_ASSERT(edge->twin->next->prev == edge->twin);
	edge->twin->next->prev = edge->prev;

	// Reset face 1 data
	b3ResetFaceData(face1);

	// Validate face 1
	Validate(face1);

	// Move face 2 conflict vertices into face 1
	qhVertex* v = face2->conflictList.head;
	while (v)
	{
		qhVertex* v0 = v;
		v = face2->conflictList.Remove(v);

		face1->conflictList.PushFront(v0);
		v0->conflictFace = face1;
	}

	// Remove face 2
	m_faceList.Remove(face2);
	FreeFace(face2);

	// Remove edge
	FreeEdge(edge->twin);
	FreeEdge(edge);

	// Repair topological errors in the face 
	while (FixFace(face1));

	// Return face 1
	return face1;
}

bool qhHull::FixFace(qhFace* face)
{
	// Maintained invariants:
	// - Each vertex must have at least three neighbor faces  
	// - Face 1 must be convex

	// Search a incoming (and outgoing edge) in the face 1
	// which have the same neighbour face.
	qhHalfEdge* edge = NULL;
	
	qhHalfEdge* ein = face->edge;
	do
	{
		qhHalfEdge* eout = ein->next;

		// Has the outgoing vertex become redundant?
		if (ein->twin->face == eout->twin->face)
		{
			edge = ein;
			break;
		}

		ein = eout;
	} while (ein != face->edge);

	if (edge)
	{
		// Remove the outgoing vertex. 
		FixMerge(face, edge);
		return true;
	}

	// Topological error not found.
	return false;
}

qhFace* qhHull::AddFace(qhVertex* v1, qhVertex* v2, qhVertex* v3)
{
	// Each vertex must be free.
	//B3_ASSERT(v1->edge == NULL);
	//B3_ASSERT(v2->edge == NULL);
	//B3_ASSERT(v3->edge == NULL);
	qhFace* face = AllocateFace();

	qhHalfEdge* e1 = FindHalfEdge(v1, v2);
	if (e1 == NULL)
	{
		e1 = AllocateEdge();
		e1->tail = v1;
		e1->face = face;
		e1->prev = NULL;
		e1->next = NULL;

		e1->twin = AllocateEdge();
		e1->twin->tail = v2;
		e1->twin->face = NULL;
		e1->twin->prev = NULL;
		e1->twin->next = NULL;
		e1->twin->twin = e1;
	}
	else
	{
		// Edge must be free.
		B3_ASSERT(e1->face == NULL);
		e1->face = face;

		B3_ASSERT(e1->tail == v1);
		B3_ASSERT(e1->twin != NULL);
		B3_ASSERT(e1->twin->active == true);
		B3_ASSERT(e1->twin->tail == v2);
	}

	qhHalfEdge* e2 = FindHalfEdge(v2, v3);
	if (e2 == NULL)
	{
		e2 = AllocateEdge();
		e2->tail = v2;
		e2->face = face;
		e2->prev = NULL;
		e2->next = NULL;

		e2->twin = AllocateEdge();
		e2->twin->tail = v3;
		e2->twin->face = NULL;
		e2->twin->prev = NULL;
		e2->twin->next = NULL;
		e2->twin->twin = e2;
	}
	else
	{
		// Edge must be free.
		B3_ASSERT(e2->face == NULL);
		e2->face = face;

		B3_ASSERT(e2->tail == v2);
		B3_ASSERT(e2->twin != NULL);
		B3_ASSERT(e2->twin->active == true);
		B3_ASSERT(e2->twin->tail == v3);
	}

	qhHalfEdge* e3 = FindHalfEdge(v3, v1);
	if (e3 == NULL)
	{
		e3 = AllocateEdge();
		e3->tail = v3;
		e3->face = face;
		e3->prev = NULL;
		e3->next = NULL;

		e3->twin = AllocateEdge();
		e3->twin->tail = v1;
		e3->twin->face = NULL;
		e3->twin->prev = NULL;
		e3->twin->next = NULL;
		e3->twin->twin = e3;
	}
	else
	{
		// Edge must be free.
		B3_ASSERT(e3->face == NULL);
		e3->face = face;

		B3_ASSERT(e3->tail == v3);
		B3_ASSERT(e3->twin != NULL);
		B3_ASSERT(e3->twin->active == true);
		B3_ASSERT(e3->twin->tail == v1);
	}

	B3_ASSERT(e1->prev == NULL);
	e1->prev = e3;
	B3_ASSERT(e1->next == NULL);
	e1->next = e2;

	B3_ASSERT(e2->prev == NULL);
	e2->prev = e1;
	B3_ASSERT(e2->next == NULL);
	e2->next = e3;

	B3_ASSERT(e3->prev == NULL);
	e3->prev = e2;
	B3_ASSERT(e3->next == NULL);
	e3->next = e1;

	face->edge = e1;

	b3ResetFaceData(face);

	face->conflictList.head = NULL;
	face->conflictList.count = 0;

	Validate(face);

	m_faceList.PushFront(face);

	return face;
}

qhFace* qhHull::RemoveFace(qhFace* face)
{
	// Conflict vertices must have been removed
	B3_ASSERT(face->conflictList.count == 0);

	// Remove half-edges 
	qhHalfEdge* e = face->edge;
	do
	{
		qhHalfEdge* e0 = e;
		e = e->next;

		// Is the edge a boundary?
		if (e0->twin->face == NULL)
		{
			// Edge is non-shared.
			FreeEdge(e0->twin);
			FreeEdge(e0);
		}
		else
		{
			// Edge is shared. 
			// Mark the twin edge as a boundary edge.
			B3_ASSERT(e0->twin != NULL);
			B3_ASSERT(e0->twin->twin == e0);
			e0->face = NULL;
			e0->prev = NULL;
			e0->next = NULL;
		}

	} while (e != face->edge);

	// Remove face 
	qhFace* nextFace = m_faceList.Remove(face);
	FreeFace(face);

	// Return the next face in the list of faces
	return nextFace;
}

bool qhHull::MergeFace(qhFace* face1)
{
	// Non-convex edge
	qhHalfEdge* edge = NULL;

	qhHalfEdge* e = face1->edge;

	do
	{
		qhHalfEdge* twin = e->twin;
		qhFace* face2 = twin->face;

		B3_ASSERT(face2 != NULL);
		B3_ASSERT(face2 != face1);

		float32 d1 = b3Distance(face2->center, face1->plane);
		float32 d2 = b3Distance(face1->center, face2->plane);

		if (d1 < -m_tolerance && d2 < -m_tolerance)
		{
			// Edge is convex
			e = e->next;
			continue;
		}

		// Edge is concave or coplanar
		edge = e;
		break;

	} while (e != face1->edge);

	if (edge)
	{
		RemoveEdge(edge);
		return true;
	}

	return false;
}

bool qhHull::MergeLargeFace(qhFace* face1)
{
	// Find a non-convex edge
	qhHalfEdge* edge = NULL;

	qhHalfEdge* e = face1->edge;

	B3_ASSERT(e->face == face1);

	do
	{
		qhHalfEdge* twin = e->twin;
		qhFace* face2 = twin->face;

		B3_ASSERT(face2 != NULL);
		B3_ASSERT(face2 != face1);

		if (face1->area > face2->area)
		{
			// Face 1 merge
			float32 d = b3Distance(face2->center, face1->plane);
			if (d < -m_tolerance)
			{
				// Edge is convex wrt to the face 1
				e = e->next;
				continue;
			}

			// Edge is concave or coplanar wrt to the face 1
			edge = e;
			break;
		}
		else
		{
			// Face 2 merge
			float32 d = b3Distance(face1->center, face2->plane);
			if (d < -m_tolerance)
			{
				// Edge is convex wrt to the face 2
				e = e->next;
				continue;
			}

			// Edge is concave or coplanar wrt to the face 2
			edge = e;
			break;
		}

		e = e->next;

	} while (e != face1->edge);

	if (edge)
	{
		RemoveEdge(edge);
		return true;
	}

	return false;
}

void qhHull::MergeNewFaces()
{
	// Merge with respect to the largest face.
	for (u32 i = 0; i < m_newFaceCount; ++i)
	{
		qhFace* face = m_newFaces[i];

		// Was the face deleted due to merging?
		if (face->active == false)
		{
			continue;
		}

		while (MergeLargeFace(face));
	}

	// Merge with respect to the both faces.
	for (u32 i = 0; i < m_newFaceCount; ++i)
	{
		qhFace* face = m_newFaces[i];

		// Was the face deleted due to merging?
		if (face->active == false)
		{
			continue;
		}

		while (MergeFace(face));
	}
}

void qhHull::Translate(const b3Vec3& translation)
{
	// Shift vertices
	for (qhVertex* v = m_vertexList.head; v != NULL; v = v->next)
	{
		v->position += translation;
	}

	// Reset face data
	for (qhFace* f = m_faceList.head; f; f = f->next)
	{
		b3ResetFaceData(f);
	}
}

void qhHull::ValidateConvexity() const
{
	for (qhFace* face = m_faceList.head; face != NULL; face = face->next)
	{
		B3_ASSERT(face->active == true);

		const qhHalfEdge* edge = face->edge;
		do
		{
			B3_ASSERT(edge->active == true);
			B3_ASSERT(edge->face == face);

			B3_ASSERT(edge->twin != NULL);
			B3_ASSERT(edge->twin->active == true);
			qhFace* other = edge->twin->face;

			// Ensure closed volume
			B3_ASSERT(other != NULL);
			B3_ASSERT(other->active == true);

			// Ensure topological health
			B3_ASSERT(face != other);

			// Ensure edge convexity
			float32 d1 = b3Distance(other->center, face->plane);
			B3_ASSERT(d1 < -m_tolerance);

			float32 d2 = b3Distance(face->center, other->plane);
			B3_ASSERT(d2 < -m_tolerance);

			// Ensure polygon convexity
			b3Vec3 P = edge->tail->position;
			b3Vec3 Q = edge->twin->tail->position;

			b3Vec3 E = Q - P;
			b3Vec3 D = b3Cross(E, face->plane.normal);

			// Edge side plane
			b3Plane plane;
			plane.normal = b3Normalize(D);
			plane.offset = b3Dot(plane.normal, P);

			// All the other vertices must be behind the edge side plane
			const qhHalfEdge* eother = edge->prev;
			do
			{
				float32 d = b3Distance(eother->tail->position, plane);
				B3_ASSERT(d <= 0.0f);

				eother = eother->prev;
			} while (eother != edge->next);

			edge = edge->next;
		} while (edge != edge);
	}
}

void qhHull::Validate(const qhHalfEdge* edge) const
{
	B3_ASSERT(edge->active == true);

	const qhHalfEdge* twin = edge->twin;
	B3_ASSERT(twin->active == true);
	B3_ASSERT(twin->twin == edge);

	B3_ASSERT(edge->tail->active == true);
	b3Vec3 A = edge->tail->position;

	B3_ASSERT(twin->tail->active == true);
	b3Vec3 B = twin->tail->position;

	B3_ASSERT(b3DistanceSquared(A, B) > B3_EPSILON * B3_EPSILON);

	const qhHalfEdge* next = edge->next;
	B3_ASSERT(next->active == true);
	B3_ASSERT(twin->tail == next->tail);

	{
		// CCW
		bool found = false;
		const qhFace* face = edge->face;
		const qhHalfEdge* e = face->edge;
		do
		{
			if (e == edge)
			{
				found = true;
				break;
			}
			e = e->next;
		} while (e != face->edge);

		B3_ASSERT(found == true);
	}

	{
		// CW
		bool found = false;
		const qhFace* face = edge->face;
		const qhHalfEdge* e = face->edge;
		do
		{
			if (e == edge)
			{
				found = true;
				break;
			}
			e = e->prev;
		} while (e != face->edge);

		B3_ASSERT(found == true);
	}
}

void qhHull::Validate(const qhFace* face) const
{
	B3_ASSERT(face->active == true);

	// CCW
	{
		const qhHalfEdge* edge = face->edge;
		do
		{
			B3_ASSERT(edge->active == true);
			B3_ASSERT(edge->face == face);

			B3_ASSERT(edge->twin != NULL);
			B3_ASSERT(edge->twin->active == true);

			if (edge->twin->face != NULL)
			{
				B3_ASSERT(edge->twin->face->active == true);
				B3_ASSERT(edge->twin->face != face);
			}

			edge = edge->next;
		} while (edge != face->edge);
	}

	// CW
	{
		const qhHalfEdge* edge = face->edge;
		do
		{
			B3_ASSERT(edge->active == true);
			B3_ASSERT(edge->face == face);

			B3_ASSERT(edge->twin != NULL);
			B3_ASSERT(edge->twin->active == true);

			if (edge->twin->face != NULL)
			{
				B3_ASSERT(edge->twin->face->active == true);
				B3_ASSERT(edge->twin->face != face);
			}

			edge = edge->prev;
		} while (edge != face->edge);
	}

	{
		const qhHalfEdge* edge = face->edge;
		do
		{
			Validate(edge);

			edge = edge->next;
		} while (edge != face->edge);
	}
}

void qhHull::Validate() const
{
	for (qhVertex* vertex = m_vertexList.head; vertex != NULL; vertex = vertex->next)
	{
		B3_ASSERT(vertex->active == true);
	}

	for (qhFace* face = m_faceList.head; face != NULL; face = face->next)
	{
		B3_ASSERT(face->active == true);

		for (qhVertex* vertex = face->conflictList.head; vertex != NULL; vertex = vertex->next)
		{
			B3_ASSERT(vertex->active == true);
		}

		// Ensure each vertex has at least three neighbor faces
		qhHalfEdge* ein = face->edge;
		do
		{
			qhHalfEdge* eout = ein->next;

			B3_ASSERT(ein->twin->face != eout->twin->face);

			ein = eout;
		} while (ein != face->edge);

		Validate(face);
	}
}

void qhHull::Draw() const
{
	for (qhFace* face = m_faceList.head; face != NULL; face = face->next)
	{
		b3Vec3 c = face->center;
		b3Vec3 n = face->plane.normal;

		b3Draw_draw->DrawSegment(c, c + n, b3Color(1.0f, 1.0f, 1.0f));
		
		const qhHalfEdge* edge = face->edge;
		do
		{
			qhVertex* v1 = face->edge->tail;
			qhVertex* v2 = edge->tail;
			const qhHalfEdge* next = edge->next;
			qhVertex* v3 = next->tail;

			b3Draw_draw->DrawSegment(v2->position, v3->position, b3Color(0.0f, 0.0f, 0.0f, 1.0f));
			b3Draw_draw->DrawSolidTriangle(n, v1->position, v2->position, v3->position, b3Color(1.0f, 1.0f, 1.0f, 0.5f));

			edge = next;
		} while (edge->next != face->edge);

		qhVertex* v = face->conflictList.head;
		while (v)
		{
			b3Draw_draw->DrawPoint(v->position, 4.0f, b3Color(1.0f, 1.0f, 0.0f));
			b3Draw_draw->DrawSegment(c, v->position, b3Color(1.0f, 1.0f, 0.0f));
			v = v->next;
		}
	}
}