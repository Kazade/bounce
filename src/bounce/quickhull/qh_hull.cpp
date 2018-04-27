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
#include <bounce/common/template/array.h>
#include <bounce/common/draw.h>

static float32 qhFindAABB(u32 iMin[3], u32 iMax[3], const b3Vec3* vertices, u32 count)
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
		b3Vec3 v = vertices[i];

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
}

qhHull::~qhHull()
{
}

void qhHull::Construct(void* memory, const b3Vec3* vs, u32 count)
{
	// Euler's formula
	// V - E + F = 2
	u32 V = count;
	u32 E = 3 * V - 6;
	u32 HE = 2 * E;
	u32 F = 2 * V - 4;

	HE *= 2;
	F *= 2;

	m_freeVertices = NULL;
	qhVertex* vertices = (qhVertex*)memory;
	for (u32 i = 0; i < V; ++i)
	{
		FreeVertex(vertices + i);
	}

	m_freeEdges = NULL;
	qhHalfEdge* edges = (qhHalfEdge*)((u8*)vertices + V * sizeof(qhVertex));
	for (u32 i = 0; i < HE; ++i)
	{
		FreeEdge(edges + i);
	}

	m_freeFaces = NULL;
	qhFace* faces = (qhFace*)((u8*)edges + HE * sizeof(qhHalfEdge));
	for (u32 i = 0; i < F; ++i)
	{
		qhFace* f = faces + i;
		f->conflictList.head = NULL;
		f->conflictList.count = 0;
		FreeFace(f);
	}

	m_horizon = (qhHalfEdge**)((u8*)faces + F * sizeof(qhFace*));
	m_horizonCount = 0;

	m_newFaces = (qhFace**)((u8*)m_horizon + HE * sizeof(qhHalfEdge*));
	m_newFaceCount = 0;

	m_vertexList.head = NULL;
	m_vertexList.count = 0;

	m_faceList.head = NULL;
	m_faceList.count = 0;
	
	m_iterations = 0;

	if (!BuildInitialHull(vs, count))
	{
		return;
	}

	qhVertex* eye = FindEyeVertex();
	while (eye)
	{
		Validate();

		AddEyeVertex(eye);

		eye = FindEyeVertex();

		++m_iterations;
	}
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
		// Store tolerance for coplanarity checks.
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
	MergeFaces();
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
	// Ensure CCW horizon order
	B3_ASSERT(m_horizonCount > 0);
	for (u32 i = 0; i < m_horizonCount; ++i)
	{
		qhHalfEdge* e1 = m_horizon[i]->twin;

		u32 j = i + 1 < m_horizonCount ? i + 1 : 0;
		qhHalfEdge* e2 = m_horizon[j];

		B3_ASSERT(e1->tail == e2->tail);
	}

	// Remove the eye vertex from the conflict list
	b3Vec3 eyePosition = eye->position;
		
	eye->conflictFace->conflictList.Remove(eye);
	FreeVertex(eye);

	// Add the eye point to the hull
	qhVertex* v1 = AddVertex(eyePosition);

	// Create new faces
	m_newFaceCount = 0;
	for (u32 i = 0; i < m_horizonCount; ++i)
	{
		qhHalfEdge* edge = m_horizon[i];

		qhVertex* v2 = edge->tail;
		qhVertex* v3 = edge->twin->tail;

		qhFace* face = AddNewFace(v1, v2, v3);
	}

	// Remove obsolete faces
	qhFace* f = m_faceList.head;
	while (f)
	{
		// Invisible faces are maintained.
		if (f->mark == qhFaceMark::e_invisible)
		{
			f = f->next;
			continue;
		}

		// Move the orphaned vertices into the new faces
		// Also remove internal points 
		qhVertex* v = f->conflictList.head;
		while (v)
		{
			b3Vec3 p = v->position;

			float32 max = m_tolerance;
			qhFace* maxFace = NULL;

			for (u32 i = 0; i < m_newFaceCount; ++i)
			{
				qhFace* newFace = m_newFaces[i];
				float32 d = b3Distance(p, newFace->plane);
				if (d > max)
				{
					max = d;
					maxFace = newFace;
				}
			}

			if (maxFace)
			{
				qhVertex* v0 = v;
				v->conflictFace = NULL;
				v = f->conflictList.Remove(v);
				maxFace->conflictList.PushFront(v0);
				v0->conflictFace = maxFace;
			}
			else
			{
				qhVertex* v0 = v;
				v->conflictFace = NULL;
				v = f->conflictList.Remove(v);
				FreeVertex(v0);
			}
		}

		// Remove the face
		f = RemoveFace(f);
	}

	// Add the new faces to the hull
	for (u32 i = 0; i < m_newFaceCount; ++i)
	{
		qhFace* face = m_newFaces[i];

		qhHalfEdge* begin = face->edge;
		qhHalfEdge* edge = begin;
		do
		{
			qhVertex* v1 = edge->tail;

			qhHalfEdge* next = edge->next;
			qhVertex* v2 = next->tail;

			edge->twin = FindHalfEdge(v2, v1);
			if (edge->twin)
			{
				edge->twin->twin = edge;
			}

			edge = next;
		} while (edge != begin);

		// Add 
		m_faceList.PushFront(face);
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

qhFace* qhHull::AddFace(qhVertex* v1, qhVertex* v2, qhVertex* v3)
{
	qhFace* face = AllocateFace();

	qhHalfEdge* e1 = FindHalfEdge(v1, v2);
	if (e1 == NULL)
	{
		e1 = AllocateEdge();
		e1->face = face;
	}

	qhHalfEdge* e2 = FindHalfEdge(v2, v3);
	if (e2 == NULL)
	{
		e2 = AllocateEdge();
		e2->face = face;
	}

	qhHalfEdge* e3 = FindHalfEdge(v3, v1);
	if (e3 == NULL)
	{
		e3 = AllocateEdge();
		e3->face = face;
	}

	e1->tail = v1;
	e1->prev = e3;
	e1->next = e2;
	e1->twin = FindHalfEdge(v2, v1);
	if (e1->twin)
	{
		e1->twin->twin = e1;
	}

	e2->tail = v2;
	e2->prev = e1;
	e2->next = e3;
	e2->twin = FindHalfEdge(v3, v2);
	if (e2->twin)
	{
		e2->twin->twin = e2;
	}

	e3->tail = v3;
	e3->prev = e2;
	e3->next = e1;
	e3->twin = FindHalfEdge(v1, v3);
	if (e3->twin)
	{
		e3->twin->twin = e3;
	}

	face->edge = e1;
	face->center = (v1->position + v2->position + v3->position) / 3.0f;
	face->plane = b3Plane(v1->position, v2->position, v3->position);

	m_faceList.PushFront(face);

	return face;
}

qhFace* qhHull::RemoveFace(qhFace* face)
{
	// Remove half-edges 
	qhHalfEdge* e = face->edge;
	do
	{
		qhHalfEdge* twin = e->twin;

		// Remove half-edge and its twin if twin face does not exist
		if (twin->face == NULL || twin->face == e->face)
		{
			e->twin = NULL;

			FreeEdge(twin);

			qhHalfEdge* e0 = e;
			e = e->next;
			FreeEdge(e0);
		}

	} while (e != face->edge);

	// Remove face 
	qhFace* nextFace = m_faceList.Remove(face);
	FreeFace(face);
	return nextFace;
}

qhFace* qhHull::AddNewFace(qhVertex* v1, qhVertex* v2, qhVertex* v3)
{
	qhFace* face = AllocateFace();

	qhHalfEdge* e1 = AllocateEdge();
	e1->face = face;
	e1->twin = NULL;
	e1->tail = v1;

	qhHalfEdge* e2 = AllocateEdge();
	e2->face = face;
	e2->twin = NULL;
	e2->tail = v2;

	qhHalfEdge* e3 = AllocateEdge();
	e3->face = face;
	e3->twin = NULL;
	e3->tail = v3;

	e1->prev = e3;
	e1->next = e2;

	e2->prev = e1;
	e2->next = e3;

	e3->prev = e2;
	e3->next = e1;

	face->edge = e1;
	face->center = (v1->position + v2->position + v3->position) / 3.0f;
	face->plane = b3Plane(v1->position, v2->position, v3->position);

	face->prev = NULL;
	face->next = NULL;

	m_newFaces[m_newFaceCount++] = face;

	return face;
}

bool qhHull::MergeFace(qhFace* rightFace)
{
	qhHalfEdge* e = rightFace->edge;

	do
	{
		qhFace* leftFace = e->twin->face;

		if (leftFace == rightFace)
		{
			e = e->next;
			continue;
		}

		float32 d1 = b3Distance(leftFace->center, rightFace->plane);
		float32 d2 = b3Distance(rightFace->center, leftFace->plane);

		if (d1 < -m_tolerance && d2 < -m_tolerance)
		{
			// Convex
			e = e->next;
			continue;
		}
		else
		{
			// Concave or coplanar

			// Move left vertices into right
			qhVertex* v = leftFace->conflictList.head;
			while (v)
			{
				qhVertex* v0 = v;
				v = leftFace->conflictList.Remove(v);
				rightFace->conflictList.PushFront(v0);
				v0->conflictFace = rightFace;
			}

			// Set right face to reference a non-deleted edge
			B3_ASSERT(e->face == rightFace);
			rightFace->edge = e->prev;

			// Absorb face
			qhHalfEdge* te = e->twin;
			do
			{
				te->face = rightFace;
				te = te->next;
			} while (te != e->twin);

			// Link edges
			e->prev->next = e->twin->next;
			e->next->prev = e->twin->prev;
			e->twin->prev->next = e->next;
			e->twin->next->prev = e->prev;

			FreeEdge(e->twin);
			FreeEdge(e);
			m_faceList.Remove(leftFace);
			FreeFace(leftFace);

			// Compute face center and plane
			rightFace->ComputeCenterAndPlane();

			// Validate
			Validate(rightFace);

			return true;
		}

	} while (e != rightFace->edge);

	return false;
}

void qhHull::MergeFaces()
{
	for (u32 i = 0; i < m_newFaceCount; ++i)
	{
		qhFace* face = m_newFaces[i];

		// Was the face deleted due to merging?
		if (face->active == false)
		{
			continue;
		}

		// Merge the faces while there is no face left to merge.
		while (MergeFace(face));
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

	u32 count = 0;
	const qhHalfEdge* begin = edge;
	do
	{
		++count;
		const qhHalfEdge* next = edge->next;
		edge = next->twin;
	} while (edge != begin);
}

void qhHull::Validate(const qhFace* face) const
{
	B3_ASSERT(face->active == true);
	
	const qhHalfEdge* begin = face->edge;
	const qhHalfEdge* edge = begin;
	do
	{
		B3_ASSERT(edge->active == true);
		B3_ASSERT(edge->face == face);
		edge = edge->next;
	} while (edge != begin);

	Validate(face->edge);
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

		Validate(face);
	}
}

void qhHull::Draw() const
{
	for (qhFace* face = m_faceList.head; face != NULL; face = face->next)
	{
		b3StackArray<b3Vec3, 256> polygon;
		polygon.Resize(0);

		const qhHalfEdge* begin = face->edge;
		const qhHalfEdge* edge = begin;
		do
		{
			polygon.PushBack(edge->tail->position);
			edge = edge->next;
		} while (edge != begin);

		b3Vec3 c = face->center;
		b3Vec3 n = face->plane.normal;

		b3Draw_draw->DrawSolidPolygon(n, polygon.Begin(), polygon.Count(), b3Color(1.0f, 1.0f, 1.0f, 0.5f));

		qhVertex* v = face->conflictList.head;
		while (v)
		{
			b3Draw_draw->DrawPoint(v->position, 4.0f, b3Color(1.0f, 1.0f, 0.0f));
			b3Draw_draw->DrawSegment(c, v->position, b3Color(1.0f, 1.0f, 0.0f));
			v = v->next;
		}

		b3Draw_draw->DrawSegment(c, c + n, b3Color(1.0f, 1.0f, 1.0f));
	}
}