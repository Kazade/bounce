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

#ifndef B3_CLOTH_PARTICLE_BODY_CONTACT_H
#define B3_CLOTH_PARTICLE_BODY_CONTACT_H

#include <bounce/common/template/list.h>
#include <bounce/common/math/vec2.h>
#include <bounce/common/math/vec3.h>
#include <bounce/common/math/transform.h>

class b3Particle;
class b3Shape;

// A contact between a particle and a body
class b3ParticleBodyContact
{
public:
private:
	friend class b3List2<b3ParticleBodyContact>;
	friend class b3Cloth;
	friend class b3Particle;
	friend class b3ClothContactManager;
	friend class b3ClothSolver;
	friend class b3ClothContactSolver;
	friend struct b3ParticleBodyContactWorldPoint;

	b3ParticleBodyContact() { }
	~b3ParticleBodyContact() { }

	void Update();

	b3Particle* m_p1;
	b3Shape* m_s2;

	bool m_active;
	
	// Contact constraint
	b3Vec3 m_normal1;
	b3Vec3 m_localPoint1;
	b3Vec3 m_localPoint2;
	float32 m_normalImpulse;

	// Friction constraint
	b3Vec3 m_tangent1, m_tangent2;
	b3Vec2 m_tangentImpulse;

	b3ParticleBodyContact* m_prev;
	b3ParticleBodyContact* m_next;
};

struct b3ParticleBodyContactWorldPoint
{
	void Initialize(const b3ParticleBodyContact* c, float32 rA, const b3Transform& xfA, float32 rB, const b3Transform& xfB);

	b3Vec3 point;
	b3Vec3 normal;
	float32 separation;
};

#endif