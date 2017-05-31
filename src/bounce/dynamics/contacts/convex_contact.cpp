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

#include <bounce/dynamics/contacts/convex_contact.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/world.h>

b3ConvexContact::b3ConvexContact(b3Shape* shapeA, b3Shape* shapeB)
{
    B3_NOT_USED(shapeA);
    B3_NOT_USED(shapeB);

	m_type = e_convexContact;

	m_manifoldCapacity = 1;
	m_manifolds = &m_stackManifold;
	m_manifoldCount = 0;

	m_cache.simplexCache.count = 0;
	m_cache.featureCache.m_featurePair.state = b3SATCacheType::e_empty;
}

bool b3ConvexContact::TestOverlap()
{
	b3Shape* shapeA = GetShapeA();
	b3Transform xfA = shapeA->GetBody()->GetTransform();

	b3Shape* shapeB = GetShapeB();
	b3Transform xfB = shapeB->GetBody()->GetTransform();

	return b3TestOverlap(xfA, 0, shapeA, xfB, 0, shapeB, &m_cache);
}

void b3ConvexContact::Collide()
{
	b3Shape* shapeA = GetShapeA();
	b3Body* bodyA = shapeA->GetBody();
	b3Transform xfA = bodyA->GetTransform();

	b3Shape* shapeB = GetShapeB();
	b3Body* bodyB = shapeB->GetBody();
	b3Transform xfB = bodyB->GetTransform();

	B3_ASSERT(m_manifoldCount == 0);
	b3CollideShapeAndShape(m_stackManifold, xfA, shapeA, xfB, shapeB, &m_cache);
	m_manifoldCount = 1;
}