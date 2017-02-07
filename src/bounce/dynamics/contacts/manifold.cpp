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

#include <bounce/dynamics/contacts/manifold.h>

void b3Manifold::GuessImpulses()
{
	pointCount = 0;
	tangentImpulse.SetZero();
	motorImpulse = 0.0f;
	for (u32 i = 0; i < B3_MAX_MANIFOLD_POINTS; ++i)
	{
		b3ManifoldPoint* p = points + i;
		p->normalImpulse = 0.0f;
		p->tangentImpulse.SetZero();
		p->persisting = 0;
	}
}

void b3Manifold::FindImpulses(const b3Manifold& oldManifold)
{
	tangentImpulse = oldManifold.tangentImpulse;
	motorImpulse = oldManifold.motorImpulse;

	for (u32 i = 0; i < oldManifold.pointCount; ++i)
	{
		const b3ManifoldPoint* p1 = oldManifold.points + i;
		for (u32 j = 0; j < pointCount; ++j)
		{
			b3ManifoldPoint* p2 = points + j;
			if (p2->triangleKey == p1->triangleKey && p2->key == p1->key)
			{
				p2->normalImpulse = p1->normalImpulse;
				p2->tangentImpulse = p1->tangentImpulse;
				p2->persisting = 1;
				break;
			}
		}
	}
}

void b3WorldManifoldPoint::Initialize(const b3ManifoldPoint* mp,
	const b3Transform& xfA, float32 radiusA,
	const b3Transform& xfB, float32 radiusB)
{
	normal = b3Mul(xfA.rotation, mp->localNormal);
	b3Vec3 p1 = b3Mul(xfA, mp->localPoint);
	b3Vec3 p2 = b3Mul(xfB, mp->localPoint2);
	point = 0.5f * (p1 + radiusA * normal + p2 - radiusB * normal);
	separation = b3Dot(p2 - p1, normal) - radiusA - radiusB;
}

void b3WorldManifold::Initialize(const b3Manifold* manifold,
	const b3Transform& xfA, float32 radiusA,
	const b3Transform& xfB, float32 radiusB)
{
	if (manifold->pointCount > 0)
	{
		center = manifold->center;
		normal = manifold->normal;
		tangent1 = manifold->tangent1;
		tangent2 = manifold->tangent2;
	}

	pointCount = manifold->pointCount;

	for (u32 i = 0; i < pointCount; ++i)
	{
		const b3ManifoldPoint* mp = manifold->points + i;
		b3WorldManifoldPoint* wmp = points + i;
		
		wmp->Initialize(mp, xfA, radiusA, xfB, radiusB);
	}
}
