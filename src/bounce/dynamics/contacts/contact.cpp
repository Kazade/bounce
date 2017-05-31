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

#include <bounce/dynamics/contacts/contact.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/world.h>
#include <bounce/dynamics/world_listeners.h>

void b3Contact::GetWorldManifold(b3WorldManifold* out, u32 index) const
{
	B3_ASSERT(index < m_manifoldCount);
	b3Manifold* m = m_manifolds + index;
	
	const b3Shape* shapeA = GetShapeA();
	const b3Body* bodyA = shapeA->GetBody();
	b3Transform xfA = bodyA->GetTransform();

	const b3Shape* shapeB = GetShapeB();
	const b3Body* bodyB = shapeB->GetBody();
	b3Transform xfB = bodyB->GetTransform();

	out->Initialize(m, shapeA->m_radius, xfA, shapeB->m_radius, xfB);
}

void b3Contact::Update(b3ContactListener* listener)
{
	b3Shape* shapeA = GetShapeA();
	b3Body* bodyA = shapeA->GetBody();
	b3Transform xfA = bodyA->GetTransform();

	b3Shape* shapeB = GetShapeB();
	b3Body* bodyB = shapeB->GetBody();
	b3Transform xfB = bodyB->GetTransform();

	b3World* world = bodyA->GetWorld();

	bool wasOverlapping = IsOverlapping();
	bool isOverlapping = false;
	bool isSensorContact = shapeA->IsSensor() || shapeB->IsSensor();

	if (isSensorContact == true)
	{
		isOverlapping = TestOverlap();
		m_manifoldCount = 0;
	}
	else
	{
		// Copy the old contact points.
		b3Manifold oldManifolds[B3_MAX_MANIFOLDS];
		u32 oldManifoldCount = m_manifoldCount;
		memcpy(oldManifolds, m_manifolds, oldManifoldCount * sizeof(b3Manifold));

		// Clear all contact points.
		m_manifoldCount = 0;
		for (u32 i = 0; i < m_manifoldCapacity; ++i)
		{
			m_manifolds[i].Initialize();
		}

		// Generate new contact points for the solver.
		Collide();

		// Initialize the new built contact points for warm starting the solver.
		if (world->m_warmStarting == true)
		{
			for (u32 i = 0; i < m_manifoldCount; ++i)
			{
				b3Manifold* m2 = m_manifolds + i;
				for (u32 j = 0; j < oldManifoldCount; ++j)
				{
					const b3Manifold* m1 = oldManifolds + j;
					m2->Initialize(*m1);
				}
			}
		}

		// The shapes are overlapping if at least one contact 
		// point was built.
		for (u32 i = 0; i < m_manifoldCount; ++i)
		{
			if (m_manifolds[i].pointCount > 0)
			{
				isOverlapping = true;
				break;
			}
		}
	}

	// Wake the bodies associated with the shapes if the contact has began.
	if (isOverlapping != wasOverlapping)
	{
		bodyA->SetAwake(true);
		bodyB->SetAwake(true);
	}

	// Update the contact state.
	if (isOverlapping == true)
	{
		m_flags |= e_overlapFlag;
	}
	else
	{
		m_flags &= ~e_overlapFlag;;
	}

	// Notify the contact listener the new contact state.
	if (listener != NULL)
	{
		if (wasOverlapping == false && isOverlapping == true)
		{
			listener->BeginContact(this);
		}

		if (wasOverlapping == true && isOverlapping == false)
		{
			listener->EndContact(this);
		}

		if (isSensorContact == false && isOverlapping == true)
		{
			listener->PreSolve(this);
		}
	}
}
