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

#ifndef B3_WORLD_LISTENERS_H
#define B3_WORLD_LISTENERS_H

#include <bounce/common/math/math.h>

class b3Shape;
class b3Contact;

class b3QueryListener 
{
public:
	virtual ~b3QueryListener() {}

	// Report to the contact listener that a shape is overlapping 
	// the queried AABB.
	virtual bool ReportShape(b3Shape* shape) = 0;
};

class b3RayCastListener 
{
public:	
	// The user must return the new ray cast fraction.
	// If fraction equals zero then the ray cast query will be canceled immediately.
	virtual ~b3RayCastListener() { }

	// Report that a shape was hit by the ray to this contact listener.
	// The reported information are the shape hit by the ray,
	// the intersection point on the shape, the surface normal associated with the point, and the 
	// intersection fraction for the ray.
	virtual float32 ReportShape(b3Shape* shape, const b3Vec3& point, const b3Vec3& normal, float32 fraction) = 0;
};

class b3ContactListener 
{
public:
	// @warning You cannot create/destroy Bounce objects inside these callbacks.
	// Inherit from this class and set it in the world to listen for collision events.	
	// Call the functions below to inspect when a shape start/end colliding with another shape.
	
	// A contact has begun.
	virtual void BeginContact(b3Contact* contact) = 0;

	// A contact has ended.
	virtual void EndContact(b3Contact* contact) = 0;
	
	// The contact will be solved after this notification.
	virtual void PreSolve(b3Contact* contact) = 0;
};

// By implementing this interface the contact filter will 
// be notified before a contact between two shapes is created and updated.
class b3ContactFilter
{
public:
	virtual ~b3ContactFilter() { }

	// Should the two shapes collide?
	virtual bool ShouldCollide(b3Shape* shapeA, b3Shape* shapeB) = 0;
};

#endif
