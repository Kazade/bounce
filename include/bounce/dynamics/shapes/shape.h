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

#ifndef B3_SHAPE_H
#define B3_SHAPE_H

#include <bounce\common\math\transform.h>
#include <bounce\common\template\list.h>
#include <bounce\collision\distance.h>

struct b3ContactEdge;

class b3Body;
class b3Shape;

enum b3ShapeType
{
	e_sphereShape,
	e_capsuleShape,
	e_hullShape,
	e_meshShape,
	e_maxShapes
};

struct b3ShapeDef 
{
	b3ShapeDef() 
	{
		shape = nullptr;
		userData = nullptr;
		isSensor = false;
		density = 0.0f;
		friction = 0.3f;
		restitution = 0.0f;
	}

	const b3Shape* shape;
	void* userData;
	bool isSensor;
	float32 density;
	float32 restitution;
	float32 friction;
};

struct b3MassData 
{
	float32 mass;
	b3Vec3 center;
	b3Mat33 I;
};

class b3Shape
{
public:
	// A shape is created and initialized by an user and a body.
	b3Shape() { }
	virtual ~b3Shape() { }

	// Get the shape type.
	b3ShapeType GetType() const;

	// Get the parent body of this shape.
	const b3Body* GetBody() const;
	b3Body* GetBody();

	// Get the frame of the shape relative to the world.
	b3Transform GetTransform() const;

	// Calculate the mass data for this shape given the shape density, that is,
	// the mass per unit volume.
	virtual void ComputeMass(b3MassData* data, float32 density) const = 0;

	// Compute the shape world AABB.
	virtual void ComputeAABB(b3AABB3* aabb, const b3Transform& xf) const = 0;

	// Test if a point is contained inside this shape.
	virtual bool TestPoint(const b3Vec3& point, const b3Transform& xf) const = 0;

	// Compute the ray intersection point, normal, and fraction.
	virtual bool RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf) const = 0;
	
	bool IsSensor() const;
	void SetSensor(bool flag);

	float32 GetDensity() const;
	void SetDensity(float32 density);

	float32 GetRestitution() const;
	void SetRestitution(float32 restitution);

	float32 GetFriction() const;
	void SetFriction(float32 friction);

	void* GetUserData() const;
	void SetUserData(void* data);

	// Destroy the contacts associated with this shape.
	void DestroyContacts();

	// Dump this shape to the log file.
	void Dump(i32 bodyIndex) const;

	// Get the next shape in the body shape list.
	const b3Shape* GetNext() const;
	b3Shape* GetNext();

	// Create a shape.
	static b3Shape* Create(const b3ShapeDef& def);
	
	// Destroy a shape.
	static void Destroy(b3Shape* shape);

	b3ShapeType m_type;
	bool m_isSensor;
	void* m_userData;
	i32 m_broadPhaseID;

	float32 m_radius;
	float32 m_density;
	float32 m_restitution;
	float32 m_friction;
	
	// Contact edges for this shape.
	b3List2<b3ContactEdge> m_contactEdges;
	
	// The parent body of this shape.
	b3Body* m_body;
	
	// Links to the body shape list.
	b3Shape* m_next;
};

inline b3ShapeType b3Shape::GetType() const 
{ 
	return m_type; 
}

inline float32 b3Shape::GetDensity() const 
{ 
	return m_density; 
}

inline void b3Shape::SetDensity(float32 density) 
{ 
	m_density = density; 
}

inline float32 b3Shape::GetRestitution() const 
{ 
	return m_restitution; 
}

inline void b3Shape::SetRestitution(float32 restitution) 
{ 
	m_restitution = restitution; 
}

inline float32 b3Shape::GetFriction() const 
{
	return m_friction;
}

inline void b3Shape::SetFriction(float32 friction) 
{
	m_friction = friction;
}

inline bool b3Shape::IsSensor() const 
{
	return m_isSensor;
}

inline void* b3Shape::GetUserData() const 
{ 
	return m_userData; 
}

inline void b3Shape::SetUserData(void* data) 
{ 
	m_userData = data; 
}

inline const b3Body* b3Shape::GetBody() const 
{
	return m_body;
}

inline b3Body* b3Shape::GetBody() 
{
	return m_body;
}

inline const b3Shape* b3Shape::GetNext() const
{
	return m_next;
}

inline b3Shape* b3Shape::GetNext()
{
	return m_next;
}

#endif
