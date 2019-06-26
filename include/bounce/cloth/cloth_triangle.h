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

#ifndef B3_CLOTH_TRIANGLE_H
#define B3_CLOTH_TRIANGLE_H

#include <bounce/cloth/cloth_collision.h>

// A cloth triangle
class b3ClothTriangle
{
public:
	// Return the triangle index.
	u32 GetTriangle() const;

	// Set the triangle radius.
	void SetRadius(float32 radius);

	// Return the triangle radius.
	float32 GetRadius() const;

	// Set the triangle coefficient of friction.
	void SetFriction(float32 friction);

	// Return the triangle coefficient of friction.
	float32 GetFriction() const;
private:
	friend class b3Cloth;
	friend class b3Particle;
	friend class b3StrechForce;
	friend class b3ClothContactManager;
	friend class b3ParticleTriangleContact;
	friend class b3ClothSolver;
	friend class b3ClothContactSolver;

	b3ClothTriangle() { }
	~b3ClothTriangle() { }

	// Synchronize AABB
	void Synchronize(const b3Vec3& displacement);

	// Cloth
	b3Cloth* m_cloth;

	// Triangle index
	u32 m_triangle;

	// Radius
	float32 m_radius;

	// Coefficient of friction
	float32 m_friction;

	// AABB Proxy
	b3ClothAABBProxy m_aabbProxy;

	// Broadphase ID
	u32 m_broadPhaseId;

	// Alpha
	float32 m_alpha;

	// Strech matrix
	float32 m_du1, m_dv1;
	float32 m_du2, m_dv2;
	float32 m_inv_det;
};

inline u32 b3ClothTriangle::GetTriangle() const
{
	return m_triangle;
}

inline void b3ClothTriangle::SetRadius(float32 radius) 
{
	m_radius = radius;
	Synchronize(b3Vec3_zero);
}

inline float32 b3ClothTriangle::GetRadius() const
{
	return m_radius;
}

inline void b3ClothTriangle::SetFriction(float32 friction)
{
	m_friction = friction;
}

inline float32 b3ClothTriangle::GetFriction() const
{
	return m_friction;
}

#endif