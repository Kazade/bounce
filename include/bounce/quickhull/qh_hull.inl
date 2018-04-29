// qhHull.h

// qhList

template<class T>
inline void qhList<T>::PushFront(T* link)
{
	link->prev = NULL;
	link->next = head;
	if (head)
	{
		head->prev = link;
	}
	head = link;
	++count;
}

template<class T>
inline T* qhList<T>::Remove(T* link)
{
	T* next = link->next;

	if (link->prev)
	{
		link->prev->next = link->next;
	}
	if (link->next)
	{
		link->next->prev = link->prev;
	}
	if (link == head)
	{
		head = link->next;
	}

	link->prev = NULL;
	link->next = NULL;

	--count;
	return next;
}

// qhHull

// Given a number of points return the required memory size in 
// bytes for constructing the convex hull of those points. 
inline u32 qhGetBufferSize(u32 pointCount)
{
	u32 size = 0;

	// Hull using Euler's Formula
	u32 V = pointCount;
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

	return size;
}

inline const qhList<qhVertex>& qhHull::GetVertexList() const
{
	return m_vertexList;
}

inline const qhList<qhFace>& qhHull::GetFaceList() const
{
	return m_faceList;
}

inline u32 qhHull::GetIterations() const
{
	return m_iterations;
}

inline qhVertex* qhHull::AllocateVertex()
{
	qhVertex* v = m_freeVertices;
	B3_ASSERT(v->active == false);
	v->active = true;
	m_freeVertices = v->freeNext;
	return v;
}

inline void qhHull::FreeVertex(qhVertex* v)
{
	//B3_ASSERT(v->active == true);
	v->active = false;
	v->freeNext = m_freeVertices;
	m_freeVertices = v;
}

inline qhHalfEdge* qhHull::AllocateEdge()
{
	qhHalfEdge* e = m_freeEdges;
	B3_ASSERT(e->active == false);
	e->active = true;
	m_freeEdges = e->freeNext;
	return e;
}

inline void qhHull::FreeEdge(qhHalfEdge* e)
{
	//B3_ASSERT(e->active == true);
	e->active = false;
	e->freeNext = m_freeEdges;
	m_freeEdges = e;
}

inline qhFace* qhHull::AllocateFace()
{
	qhFace* f = m_freeFaces;
	B3_ASSERT(f->active == false);
	f->active = true;
	m_freeFaces = f->freeNext;
	return f;
}

inline void qhHull::FreeFace(qhFace* f)
{
	//B3_ASSERT(f->active == true);
	f->active = false;
	f->freeNext = m_freeFaces;
	m_freeFaces = f;
}

inline qhHalfEdge* qhHull::FindHalfEdge(const qhVertex* v1, const qhVertex* v2) const
{
	for (qhFace* face = m_faceList.head; face != NULL; face = face->next)
	{
		qhHalfEdge* e = face->edge;
		do
		{
			if (e->tail == v1 && e->next->tail == v2)
			{
				return e;
			}

			if (e->tail == v2 && e->next->tail == v1)
			{
				return e->twin;
			}

			e = e->next;
		} while (e != face->edge);
	}
	return NULL;
}