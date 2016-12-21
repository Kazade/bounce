// qhHull.h

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

inline u32 qhFace::VertexCount() const
{
	u32 count = 0;
	qhHalfEdge* e = edge;
	do
	{
		++count;
		e = e->next;
	} while (e != edge);
	return count;
}

inline u32 qhFace::EdgeCount() const
{
	u32 count = 0;
	qhHalfEdge* e = edge;
	do
	{
		++count;
		e = e->next;
	} while (e != edge);
	return count;
}

inline qhHalfEdge* qhFace::FindTwin(const qhVertex* tail, const qhVertex* head) const
{
	qhHalfEdge* e = edge;
	do
	{
		qhVertex* tail2 = e->tail;
		qhVertex* head2 = e->next->tail;

		if (tail2 == tail && head2 == head)
		{
			return e;
		}

		e = e->next;
	} while (e != edge);

	return NULL;
}

inline b3Vec3 b3Newell(const b3Vec3& a, const b3Vec3& b)
{
	return b3Vec3((a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y));
}

inline void qhFace::ComputeCenterAndPlane()
{
	b3Vec3 c;
	c.SetZero();

	b3Vec3 n;
	n.SetZero();

	u32 count = 0;
	qhHalfEdge* e = edge;
	do
	{
		b3Vec3 v1 = e->tail->position;
		b3Vec3 v2 = e->next->tail->position;

		n += b3Newell(v1, v2);
		c += v1;

		++count;
		e = e->next;
	} while (e != edge);

	B3_ASSERT(count > 0);
	c /= float32(count);
	n.Normalize();
	plane.normal = n;
	plane.offset = b3Dot(n, c);
	center = c;
}

inline qhHalfEdge* qhHull::FindTwin(const qhVertex* tail, const qhVertex* head) const
{
	qhFace* face = m_faceList.head;
	while (face)
	{
		qhHalfEdge* e = face->FindTwin(tail, head);
		if (e)
		{
			return e;
		}
		face = face->next;
	}
	return NULL;
}

inline u32 qhGetMemorySize(u32 V)
{
	u32 E = 3 * V - 6;
	u32 HE = 2 * E;
	u32 F = 2 * V - 4;

	// V - E + F = 2
	
	HE *= 2;
	F *= 2;

	u32 size = 0;
	size += V * sizeof(qhVertex);
	size += HE * sizeof(qhHalfEdge);
	size += F * sizeof(qhFace);
	return size;
}

inline qhVertex* qhHull::AllocateVertex()
{
	qhVertex* v = m_freeVertices;
	m_freeVertices = v->freeNext;
	return v;
}

inline void qhHull::FreeVertex(qhVertex* v)
{
	v->freeNext = m_freeVertices;
	m_freeVertices = v;
}

inline qhHalfEdge* qhHull::AllocateEdge()
{
	qhHalfEdge* e = m_freeEdges;
	m_freeEdges = e->freeNext;
	return e;
}

inline void qhHull::FreeEdge(qhHalfEdge* e)
{
	e->freeNext = m_freeEdges;
	m_freeEdges = e;
}

inline qhFace* qhHull::AllocateFace()
{
	qhFace* f = m_freeFaces;
	m_freeFaces = f->freeNext;
	return f;
}

inline void qhHull::FreeFace(qhFace* f)
{
	f->state = qhFace::e_deleted;
	f->freeNext = m_freeFaces;
	m_freeFaces = f;
}