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
	B3_ASSERT(m_vertexCount < m_vertexCapacity);
	++m_vertexCount;

	qhVertex* v = m_freeVertices;
	B3_ASSERT(v->active == false);
	v->active = true;
	m_freeVertices = v->freeNext;
	return v;
}

inline void qhHull::FreeVertex(qhVertex* v)
{
	B3_ASSERT(m_vertexCount > 0);
	--m_vertexCount;

	B3_ASSERT(v->active == true);
	v->active = false;
	v->freeNext = m_freeVertices;
	m_freeVertices = v;
}

inline qhHalfEdge* qhHull::AllocateEdge()
{
	B3_ASSERT(m_edgeCount < m_edgeCapacity);
	++m_edgeCount;

	qhHalfEdge* e = m_freeEdges;
	B3_ASSERT(e->active == false);
	e->active = true;
	m_freeEdges = e->freeNext;
	return e;
}

inline void qhHull::FreeEdge(qhHalfEdge* e)
{
	B3_ASSERT(m_edgeCount > 0);
	--m_edgeCount;

	B3_ASSERT(e->active == true);
	e->active = false;
	e->freeNext = m_freeEdges;
	m_freeEdges = e;
}

inline qhFace* qhHull::AllocateFace()
{
	B3_ASSERT(m_faceCount < m_faceCapacity);
	++m_faceCount;

	qhFace* f = m_freeFaces;
	B3_ASSERT(f->active == false);
	f->active = true;
	m_freeFaces = f->freeNext;
	return f;
}

inline void qhHull::FreeFace(qhFace* f)
{
	B3_ASSERT(m_faceCount > 0);
	--m_faceCount;
	
	B3_ASSERT(f->active == true);
	f->active = false;
	f->freeNext = m_freeFaces;
	m_freeFaces = f;
}