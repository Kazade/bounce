/*
* Copyright (c) 2016-2019 Irlan Robson https://irlanrobson.github.io
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

#ifndef B3_CLOTH_SHAPE_H
#define B3_CLOTH_SHAPE_H

#include <bounce/common/settings.h>

class b3Cloth;

// Cloth shape type
enum b3ClothShapeType
{
	e_clothSphereShape,
	e_clothCapsuleShape,
	e_clothTriangleShape,
	e_clothWorldShape,
	e_maxClothShapes
};

// Cloth shape definition
struct b3ClothShapeDef
{
	b3ClothShapeDef()
	{
		radius = scalar(0);
		friction = scalar(0);
		density = scalar(0);
		meshIndex = B3_MAX_U32;
	}

	scalar radius;
	scalar friction;
	scalar density;
	u32 meshIndex;
};

// Cloth shape
class b3ClothShape
{
public:
	// Get the shape type.
	b3ClothShapeType GetType() const;

	// Get the cloth.
	b3Cloth* GetCloth();
	const b3Cloth* GetCloth() const;

	// Get the shape radius.
	scalar GetRadius() const;

	// Get the shape density.
	scalar GetDensity() const;

	// Set the coefficient of friction of this shape.
	void SetFriction(scalar friction);

	// Get the coefficient of friction of this shape.
	// This represents both static and dynamic friction.
	scalar GetFriction() const;
protected:
	friend class b3Cloth;
	friend class b3ClothParticle;
	friend class b3ClothContactManager;
	friend class b3ClothContactSolver;

	// Type
	b3ClothShapeType m_type;

	// Cloth
	b3Cloth* m_cloth;

	// Radius
	scalar m_radius;

	// Coefficient of friction
	scalar m_friction;

	// Density
	scalar m_density;

	// Broadphase ID
	u32 m_broadPhaseId;

	// Mesh index
	u32 m_meshIndex;
};

inline b3ClothShapeType b3ClothShape::GetType() const
{
	return m_type;
}

inline b3Cloth* b3ClothShape::GetCloth()
{
	return m_cloth;
}

inline const b3Cloth* b3ClothShape::GetCloth() const
{
	return m_cloth;
}

inline scalar b3ClothShape::GetRadius() const
{
	return m_radius;
}

inline void b3ClothShape::SetFriction(scalar friction)
{
	m_friction = friction;
}

inline scalar b3ClothShape::GetFriction() const
{
	return m_friction;
}

inline scalar b3ClothShape::GetDensity() const
{
	return m_density;
}

#endif