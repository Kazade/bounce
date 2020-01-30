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

#include <bounce/cloth/cloth_particle.h>
#include <bounce/cloth/shapes/cloth_sphere_shape.h>
#include <bounce/cloth/shapes/cloth_capsule_shape.h>
#include <bounce/cloth/shapes/cloth_triangle_shape.h>
#include <bounce/cloth/cloth.h>
#include <bounce/cloth/forces/force.h>

b3ClothParticle::b3ClothParticle(const b3ClothParticleDef& def, b3Cloth* cloth)
{
	m_cloth = cloth;
	m_type = def.type;
	m_position = def.position;
	m_velocity = def.velocity;
	m_force = def.force;
	m_translation.SetZero();

	if (m_type == e_dynamicClothParticle)
	{
		m_mass = scalar(1);
		m_invMass = scalar(1);
	}
	else
	{
		m_mass = scalar(0);
		m_invMass = scalar(0);
	}

	m_x.SetZero();
	m_meshIndex = def.meshIndex;
	m_userData = def.userData;
}

b3ClothParticle::~b3ClothParticle()
{

}

void b3ClothParticle::SetType(b3ClothParticleType type)
{
	if (m_type == type)
	{
		return;
	}

	m_type = type;
	
	if (m_type == e_staticClothParticle || m_type == e_kinematicClothParticle)
	{
		m_mass = scalar(0);
		m_invMass = scalar(0);
	}
	else
	{
		m_cloth->ResetMass();
	}

	m_force.SetZero();

	if (type == e_staticClothParticle)
	{
		m_velocity.SetZero();
		m_translation.SetZero();

		SynchronizeSpheres();
		SynchronizeCapsules();
		SynchronizeTriangles();
	}
}

void b3ClothParticle::DestroySpheres()
{
	b3ClothSphereShape* s = m_cloth->m_sphereShapeList.m_head;
	while (s)
	{
		b3ClothSphereShape* s0 = s;
		s = s->m_next;

		if (s0->m_p == this)
		{
			m_cloth->DestroySphereShape(s0);
		}
	}
}

void b3ClothParticle::DestroyCapsules()
{
	b3ClothCapsuleShape* c = m_cloth->m_capsuleShapeList.m_head;
	while (c)
	{
		b3ClothCapsuleShape* c0 = c;
		c = c->m_next;

		if (c0->m_p1 == this || c0->m_p2 == this)
		{
			m_cloth->DestroyCapsuleShape(c0);
		}
	}
}

void b3ClothParticle::DestroyTriangles()
{
	b3ClothTriangleShape* t = m_cloth->m_triangleShapeList.m_head;
	while (t)
	{
		b3ClothTriangleShape* t0 = t;
		t = t->m_next;

		if (t0->m_p1 == this || t0->m_p2 == this || t0->m_p3 == this)
		{
			m_cloth->DestroyTriangleShape(t0);
		}
	}
}

void b3ClothParticle::DestroyForces()
{
	b3Force* f = m_cloth->m_forceList.m_head;
	while (f)
	{
		b3Force* f0 = f;
		f = f->m_next;

		if (f0->HasParticle(this))
		{
			m_cloth->DestroyForce(f0);
		}
	}
}

void b3ClothParticle::DestroyContacts()
{
	{
		// Destroy shape contacts
		b3ClothSphereAndShapeContact* c = m_cloth->m_contactManager.m_sphereAndShapeContactList.m_head;
		while (c)
		{
			if (c->m_s1->m_p == this)
			{
				b3ClothSphereAndShapeContact* quack = c;
				c = c->m_next;
				m_cloth->m_contactManager.Destroy(quack);
				continue;
			}

			c = c->m_next;
		}
	}

	{
		// Destroy capsule contacts
		b3ClothCapsuleAndCapsuleContact* c = m_cloth->m_contactManager.m_capsuleAndCapsuleContactList.m_head;
		while (c)
		{
			if (c->m_s1->m_p1 == this ||
				c->m_s1->m_p2 == this ||
				c->m_s2->m_p1 == this ||
				c->m_s2->m_p2 == this)
			{
				b3ClothCapsuleAndCapsuleContact* quack = c;
				c = c->m_next;
				m_cloth->m_contactManager.Destroy(quack);
				continue;
			}

			c = c->m_next;
		}
	}

	{
		// Destroy triangle contacts
		b3ClothSphereAndTriangleContact* c = m_cloth->m_contactManager.m_sphereAndTriangleContactList.m_head;
		while (c)
		{
			if (c->m_s1->m_p == this ||
				c->m_s2->m_p1 == this ||
				c->m_s2->m_p2 == this ||
				c->m_s2->m_p3 == this)
			{
				b3ClothSphereAndTriangleContact* quack = c;
				c = c->m_next;
				m_cloth->m_contactManager.Destroy(quack);
				continue;
			}

			c = c->m_next;
		}
	}
}

void b3ClothParticle::SynchronizeSpheres()
{
	for (b3ClothSphereShape* s = m_cloth->m_sphereShapeList.m_head; s; s = s->m_next)
	{
		if (s->m_p == this)
		{
			s->Synchronize(b3Vec3_zero);
		}
	}
}

void b3ClothParticle::SynchronizeCapsules()
{
	for (b3ClothCapsuleShape* c = m_cloth->m_capsuleShapeList.m_head; c; c = c->m_next)
	{
		if (c->m_p1 == this || c->m_p2 == this)
		{
			c->Synchronize(b3Vec3_zero);
		}
	}
}

void b3ClothParticle::SynchronizeTriangles()
{
	for (b3ClothTriangleShape* t = m_cloth->m_triangleShapeList.m_head; t; t = t->m_next)
	{
		if (t->m_p1 == this || t->m_p2 == this || t->m_p3 == this)
		{
			t->Synchronize(b3Vec3_zero);
		}
	}
}