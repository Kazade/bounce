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

#ifndef QH_HULL_H
#define QH_HULL_H

#include <bounce/common/geometry.h>

template<class T>
struct qhList
{
	void PushFront(T* link);
	T* Remove(T* link);

	T* head;
	u32 count;
};

struct qhHalfEdge;
struct qhVertex;

struct qhFace
{
	enum State
	{
		e_invisible,
		e_visible,
		e_unknown,
		e_deleted
	};

	qhFace* freeNext;

	qhFace* prev;
	qhFace* next;

	qhHalfEdge* edge;

	qhList<qhVertex> conflictList;
	
	State state;
	b3Vec3 center;
	b3Plane plane;

	u32 GetVertexCount() const;
	u32 GetEdgeCount() const;
	qhHalfEdge* FindTwin(const qhVertex* tail, const qhVertex* head) const;
	void ComputeCenterAndPlane();
};

struct qhHalfEdge
{
	qhHalfEdge* freeNext;

	qhVertex* tail;

	qhHalfEdge* prev;
	qhHalfEdge* next;
	qhHalfEdge* twin;

	qhFace* face;
};

struct qhVertex
{
	qhVertex* freeNext;

	qhVertex* prev;
	qhVertex* next;

	b3Vec3 position;

	qhFace* conflictFace;
};

// A convex hull builder. 
// Given a list of points constructs its convex hull. 
// The output convex hull might contain polygonal faces and not only triangles. 
// Coplanar face merging is necessary for stable physics simulation.
class qhHull
{
public:
	qhHull();
	~qhHull();
	
	// Construct this hull given a memory buffer and an array of points.
	// Use qhGetBufferSize to get the buffer size given the number of points.
	void Construct(void* buffer, const b3Vec3* vertices, u32 vertexCount);

	// Get the number of iterations this algorithm ran.
	u32 GetIterations() const;

	// Get the list of faces in this convex hull.
	const qhList<qhFace>& GetFaceList() const;

	// Validate this hull.
	void Validate() const;
	void Validate(const qhFace* face) const;
	void Validate(const qhHalfEdge* edge) const;

	// Draw this hull.
	void Draw() const;
private:
	bool BuildInitialHull(const b3Vec3* vertices, u32 count);

	qhVertex* FindEyeVertex() const;
	void AddVertex(qhVertex* v);

	void FindHorizon(qhVertex* eye);

	void AddNewFaces(qhVertex* eye);
	void MergeFaces();
	bool MergeFace(qhFace* face);

	qhFace* CreateTriangle(qhVertex* v1, qhVertex* v2, qhVertex* v3);
	qhHalfEdge* CreateAdjoiningTriangle(qhVertex* v, qhHalfEdge* he);
	
	qhHalfEdge* FindTwin(const qhVertex* tail, const qhVertex* head) const;

	// Coplanarity tolerance
	float32 m_tolerance;

	// Number of Quickhull iterations
	u32 m_iterations;

	// List of faces
	qhList<qhFace> m_faceList; 
	
	// Memory
	qhVertex* AllocateVertex();
	void FreeVertex(qhVertex* p);
	
	qhHalfEdge* AllocateEdge();
	void FreeEdge(qhHalfEdge* p);
	
	qhFace* AllocateFace();
	void FreeFace(qhFace* p);
	
	qhVertex* m_freeVertices;
	qhHalfEdge* m_freeEdges;
	qhFace* m_freeFaces;
	
	qhHalfEdge** m_horizon;
	u32 m_horizonCount;

	qhFace** m_newFaces;
	u32 m_newFaceCount;
};

#include <bounce/quickhull/qh_hull.inl>

#endif