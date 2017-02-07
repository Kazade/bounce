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

#ifndef B3_MANIFOLD_H
#define B3_MANIFOLD_H

#include <bounce/common/math/vec2.h>
#include <bounce/common/geometry.h>

#define B3_NULL_TRIANGLE (0xFFFFFFFF)

// A contact manifold point.
struct b3ManifoldPoint
{
	u32 triangleKey; // triangle identifier
	u32 key; // point identifier
	b3Vec3 localNormal; // local normal on the first shape
	b3Vec3 localPoint; // local point on the first shape	
	b3Vec3 localPoint2; // local point on the other shape
	float32 normalImpulse; // normal impulse
	b3Vec2 tangentImpulse; // tangent impulses 
	u8 persisting; // indicates that the point is persisting
};

// A manifold is a group of contact points with similar contact normal.
struct b3Manifold
{
	// Choose arbitrary impulses for warm starting.
	void GuessImpulses();
	
	// Initialize impulses for warm starting.
	void FindImpulses(const b3Manifold& old);
	
	b3ManifoldPoint points[B3_MAX_MANIFOLD_POINTS]; // manifold points
	u32 pointCount; // number of manifold points

	b3Vec3 center;
	b3Vec3 normal;
	b3Vec3 tangent1;
	b3Vec3 tangent2;
	b3Vec2 tangentImpulse;
	float32 motorImpulse;
};

// A world manifold point.
struct b3WorldManifoldPoint
{
	// Initialize this manifold from a local manifold point and two transforms.
	// The radii should come from the shapes that generated the manifold.
	void Initialize(const b3ManifoldPoint* point,
		const b3Transform& xfA, float32 radiusA,
		const b3Transform& xfB, float32 radiusB);

	b3Vec3 point;
	b3Vec3 normal;
	b3Vec2 tangents[2];
	float32 separation;
};

// A contact manifold is a group of contact points with similar normal.
struct b3WorldManifold
{
	// Initialize this world manifold from a local manifold and two transforms.
	// The radii should come from the shapes that generated the manifold.
	void Initialize(const b3Manifold* manifold,
		const b3Transform& xfA, float32 radiusA,
		const b3Transform& xfB, float32 radiusB);

	b3Vec3 center;
	b3Vec3 normal;
	b3Vec3 tangent1;
	b3Vec3 tangent2;

	b3WorldManifoldPoint points[B3_MAX_MANIFOLD_POINTS]; // contact points
	u32 pointCount; // number of contact points
};

#endif
