inline b3HalfEdge b3MakeEdge(u32 origin, u32 twin, u32 face, u32 next)
{
	b3HalfEdge edge;
	edge.origin = origin;
	edge.twin = twin;
	edge.face = face;
	edge.next = next;
	return edge;
}

inline const b3Vec3& b3Hull::GetVertex(u32 index) const
{
	return vertices[index];
}

inline const b3HalfEdge* b3Hull::GetEdge(u32 index) const
{
	return edges + index;
}

inline const b3Face* b3Hull::GetFace(u32 index) const
{
	return faces + index;
}

inline const b3Plane& b3Hull::GetPlane(u32 index) const
{
	return planes[index];
}

inline u32 b3Hull::GetSupportVertex(const b3Vec3& direction) const
{
	u32 maxIndex = 0;
	float32 maxProjection = b3Dot(direction, vertices[maxIndex]);
	for (u32 i = 1; i < vertexCount; ++i)
	{
		float32 projection = b3Dot(direction, vertices[i]);
		if (projection > maxProjection)
		{
			maxIndex = i;
			maxProjection = projection;
		}
	}
	return maxIndex;
}

inline u32 b3Hull::GetSupportFace(const b3Vec3& direction) const
{
	u32 maxIndex = 0;
	float32 maxProjection = b3Dot(direction, planes[maxIndex].normal);
	for (u32 i = 1; i < faceCount; ++i)
	{
		float32 projection = b3Dot(direction, planes[i].normal);
		if (projection > maxProjection)
		{
			maxIndex = i;
			maxProjection = projection;
		}
	}
	return maxIndex;
}

inline b3Plane b3Hull::GetEdgeSidePlane(u32 index) const
{
	const b3HalfEdge* edge = edges + index;
	const b3HalfEdge* twin = edges + edge->twin;
	const b3Plane* facePlane = planes + edge->face;

	b3Vec3 P = vertices[edge->origin];
	b3Vec3 Q = vertices[twin->origin];
	b3Vec3 E = Q - P;
	b3Vec3 D = b3Cross(E, facePlane->normal);

	b3Plane plane;
	plane.normal = b3Normalize(D);
	plane.offset = b3Dot(plane.normal, P);
	return plane;
}

inline u32 b3Hull::GetSize() const
{
	u32 size = 0;
	size += sizeof(b3Hull);
	size += vertexCount * sizeof(b3Vec3);
	size += edgeCount * sizeof(b3HalfEdge);
	size += faceCount * sizeof(b3Face);
	size += faceCount * sizeof(b3Plane);
	return size;
}