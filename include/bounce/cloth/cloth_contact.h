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

#ifndef B3_CLOTH_CONTACT_H
#define B3_CLOTH_CONTACT_H

#include <bounce/common/template/list.h>

class b3Particle;
struct b3ClothMeshTriangle;
struct b3ClothAABBProxy;

// Contact between particle and a triangle
class b3ParticleTriangleContact
{
public:
private:
	friend class b3Cloth;
	friend class b3Particle;
	friend class b3ClothContactManager;
	friend class b3List2<b3ParticleTriangleContact>;
	friend class b3ClothContactSolver;

	b3ParticleTriangleContact() { }
	~b3ParticleTriangleContact() { }

	void Update();

	// Particle
	b3Particle* m_p1;

	// Triangle
	b3ClothMeshTriangle* m_triangle;
	b3ClothAABBProxy* m_triangleProxy;
	b3Particle* m_p2;
	b3Particle* m_p3;
	b3Particle* m_p4;

	float32 m_normalImpulse;

	bool m_front;

	bool m_active;

	b3ParticleTriangleContact* m_prev;
	b3ParticleTriangleContact* m_next;
};

#endif