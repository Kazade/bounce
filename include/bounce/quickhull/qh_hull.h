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

#include <bounce\common\geometry.h>
#include <bounce\common\template\array.h>

template<class T>
struct qhList
{
	void PushFront(T* link);
	T* Remove(T* link);

	T* head;
	u32 count;
};

// Half-edge data structure definition used by qhHull.
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

	u32 VertexCount() const;
	u32 EdgeCount() const; 
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

// todo
// Snapshots of the algorithm for debug drawing.
struct qhDraw
{
	//DrawIteration* iter; // current iteration
	//b3Array<DrawIteration> iterations;
};

class b3Draw;

// Given a number of points return the required memory size in bytes for constructing the 
// convex hull of those points. Use this function before allocating the memory buffer passed 
// as argument to Construct.
u32 qhGetMemorySize(u32 V);

// A convex hull builder. Given a list of points constructs its convex hull. 
// The output convex hull might contain polygonal faces and not only triangles. 
// Coplanar face merging is necessary for stable physics simulation.
class qhHull
{
public:
	qhHull();
	~qhHull();

	// Entry point of qhHull.
	// Construct this hull given a memory buffer and a list of points.
	// Use qhGetMemorySize to see how many free bytes should be available in the buffer.
	void Construct(void* memory, const b3Array<b3Vec3>& vertices);

	// Output of qhHull.
	// todo 
	// Output a cleaner data structure. Maybe similar to b3Hull but storing larger hulls?
	qhList<qhFace> m_faceList; // convex hull
	u32 m_iteration; // number of quickhull iterations

	bool IsConsistent() const;
	
	void Draw(b3Draw* draw) const;
private:
	bool BuildInitialHull(const b3Array<b3Vec3>& vertices);

	qhVertex* NextVertex();
	
	void AddVertex(qhVertex* v);
	
	void BuildHorizon(b3Array<qhHalfEdge*>& horizon, qhVertex* eye);
	
	void BuildHorizon(b3Array<qhHalfEdge*>& horizon, qhVertex* eye, qhHalfEdge* e0, qhFace* f);
	
	qhFace* AddTriangle(qhVertex* v1, qhVertex* v2, qhVertex* v3);
	
	qhHalfEdge* AddAdjoiningTriangle(qhVertex* v, qhHalfEdge* he);
	
	void AddNewFaces(b3Array<qhFace*>& newFaces, qhVertex* eye, const b3Array<qhHalfEdge*>& horizon);

	bool MergeFace(qhFace* face);
	
	void MergeFaces(b3Array<qhFace*>& newFaces);

	qhHalfEdge* FindTwin(const qhVertex* tail, const qhVertex* head) const;

	// Coplanarity tolerance
	float32 m_tolerance;
	
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
};

#include <bounce/quickhull/qh_hull.inl>

#endif