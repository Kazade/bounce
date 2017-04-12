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

#ifndef B3_CONTACT_H
#define B3_CONTACT_H

#include <bounce/common/math/math.h>
#include <bounce/common/template/list.h>
#include <bounce/common/template/array.h>
#include <bounce/dynamics/contacts/manifold.h>

class b3Shape;
class b3Body;
class b3Contact;
class b3ContactListener;

// A contact edge for the contact graph, 
// where a shape is a vertex and a contact 
// an edge.
struct b3ContactEdge
{
	b3Shape* other;
	b3Contact* contact;
	// Links to the contact edge list.
	b3ContactEdge* m_prev;
	b3ContactEdge* m_next;
};

// This goes inside a contact.
// It holds two shapes that are overlapping.
struct b3OverlappingPair
{
	// To the shape A edge list.
	b3Shape* shapeA;
	b3ContactEdge edgeA;
	// To the shape B edge list.
	b3Shape* shapeB;
	b3ContactEdge edgeB;
};

// todo
struct b3TOIEvent
{
	float32 t;
};

enum b3ContactType
{
	e_convexContact,
	e_meshContact,
	e_maxContact
};

class b3Contact
{
public:
	// Get the contact type.
	b3ContactType GetType() const;

	// Get the shape A in this contact.
	const b3Shape* GetShapeA() const;
	b3Shape* GetShapeA();

	// Get the shape B in this contact.
	const b3Shape* GetShapeB() const;
	b3Shape* GetShapeB();

	// Get the manifold capacity from this contact.
	u32 GetManifoldCapacity() const;
	
	// Get a contact manifold from this contact.
	const b3Manifold* GetManifold(u32 index) const;
	b3Manifold* GetManifold(u32 index);

	// Get the number of manifolds from this contact.
	u32 GetManifoldCount() const;

	// Get a world contact manifold from this contact.
	void GetWorldManifold(b3WorldManifold* out, u32 index) const;
	
	// Are the shapes in this contact overlapping?
	bool IsOverlapping() const;

	// Get the next contact in the world contact list.
	const b3Contact* GetNext() const;
	b3Contact* GetNext();
protected:
	friend class b3World;
	friend class b3Island;
	friend class b3Shape;
	friend class b3ContactManager;
	friend class b3ContactSolver;
	friend class b3List2<b3Contact>;

	enum b3ContactFlags 
	{
		e_overlapFlag = 0x0001,
		e_islandFlag = 0x0002,
	};

	b3Contact() { }
	virtual ~b3Contact() { }

	// Update the contact state.
	void Update(b3ContactListener* listener);

	// Test if the shapes in this contact are overlapping.
	virtual bool TestOverlap() = 0;

	// Initialize contact constraits.
	virtual void Collide() = 0;

	b3ContactType m_type;
	u32 m_flags;
	b3OverlappingPair m_pair;

	// Collision event from discrete collision to 
	// discrete physics.
	u32 m_manifoldCapacity;
	b3Manifold* m_manifolds;
	u32 m_manifoldCount;

	// Time of impact event from continuous collision
	// to continuous physics.
	//b3TOIEvent m_toi;

	// Links to the world contact list.
	b3Contact* m_prev;
	b3Contact* m_next;
};

inline b3ContactType b3Contact::GetType() const
{
	return m_type;
}

inline b3Shape* b3Contact::GetShapeA() 
{
	return m_pair.shapeA;
}

inline const b3Shape* b3Contact::GetShapeA() const 
{
	return m_pair.shapeA;
}

inline b3Shape* b3Contact::GetShapeB() 
{
	return m_pair.shapeB;
}

inline const b3Shape* b3Contact::GetShapeB() const 
{
	return m_pair.shapeB;
}

inline u32 b3Contact::GetManifoldCapacity() const
{
	return m_manifoldCapacity;
}

inline const b3Manifold* b3Contact::GetManifold(u32 index) const
{
	B3_ASSERT(index < m_manifoldCount);
	return m_manifolds + index;
}

inline b3Manifold* b3Contact::GetManifold(u32 index)
{
	B3_ASSERT(index < m_manifoldCount);
	return m_manifolds + index;
}

inline u32 b3Contact::GetManifoldCount() const
{
	return m_manifoldCount;
}

inline bool b3Contact::IsOverlapping() const 
{
	return (m_flags & e_overlapFlag) != 0;
}

inline const b3Contact* b3Contact::GetNext() const
{
	return m_next;
}

inline b3Contact* b3Contact::GetNext()
{
	return m_next;
}

#endif