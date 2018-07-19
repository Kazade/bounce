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

#ifndef B3_HULL_H
#define B3_HULL_H

#include <bounce/common/geometry.h>

struct b3Face
{
	u32 edge;
};

struct b3HalfEdge
{
	u32 origin;
	u32 twin;
	u32 face;
	u32 next;
};

struct b3Hull
{
	b3Vec3 centroid;
	u32 vertexCount;
	b3Vec3* vertices;
	u32 edgeCount;
	b3HalfEdge* edges;
	u32 faceCount;
	b3Face* faces;
	b3Plane* planes;
	
	const b3Vec3& GetVertex(u32 index) const;
	const b3HalfEdge* GetEdge(u32 index) const;
	const b3Face* GetFace(u32 index) const;
	const b3Plane& GetPlane(u32 index) const;

	u32 GetSupportVertex(const b3Vec3& direction) const;
	//u32 GetSupportEdge(const b3Vec3& direction) const;
	u32 GetSupportFace(const b3Vec3& direction) const;
	
	b3Plane GetEdgeSidePlane(u32 index) const;
	
	u32 GetSize() const;

	void Validate() const;
	void Validate(const b3Face* face) const;
	void Validate(const b3HalfEdge* edge) const;
};

#include <bounce/collision/shapes/hull.inl>

#endif