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

#ifndef B3_MESH_CONTACT_H
#define B3_MESH_CONTACT_H

#include <bounce\collision\shapes\aabb3.h>
#include <bounce\dynamics\contacts\contact.h>
#include <bounce\dynamics\contacts\manifold.h>
#include <bounce\dynamics\contacts\collide\collide.h>

struct b3TriangleCache
{
	u32 index;
	b3ConvexCache cache;
};

class b3MeshContact;

struct b3MeshContactLink
{
	b3MeshContact* m_c;
	b3MeshContactLink* m_prev;
	b3MeshContactLink* m_next;
};

class b3MeshContact : public b3Contact
{
public:
private:
	friend class b3ContactManager;
	friend class b3List2<b3MeshContact>;
	friend class b3StaticTree;

	b3MeshContact(b3Shape* shapeA, b3Shape* shapeB);
	~b3MeshContact();

	bool TestOverlap();

	void Collide();
	
	void SynchronizeShapes();

	bool MoveAABB(const b3AABB3& aabb, const b3Vec3& displacement);

	void FindNewPairs();

	bool Report(u32 proxyId);

	// Did the AABB move significantly?
	bool m_aabbMoved;

	// The first shape AABB in the frame of the other shape.
	b3AABB3 m_aabbA; 
	
	// Child shapes potentially overlapping with 
	// the first shape.
	u32 m_triangleCapacity;
	b3TriangleCache* m_triangles;
	u32 m_triangleCount;

	// Contact manifolds.
	b3Manifold m_stackManifolds[B3_MAX_MANIFOLDS];

	// Links to the world mesh contact list.
	b3MeshContactLink m_link;
};

#endif
