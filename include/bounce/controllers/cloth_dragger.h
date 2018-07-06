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

#ifndef B3_CLOTH_DRAGGER_H
#define B3_CLOTH_DRAGGER_H

#include <bounce/collision/collision.h>
#include <bounce/dynamics/cloth/cloth.h>
#include <bounce/dynamics/cloth/cloth_mesh.h>
#include <bounce/dynamics/cloth/particle.h>
#include <bounce/dynamics/cloth/spring_force.h>
#include <bounce/dynamics/world.h>

// A cloth triangle dragger.
class b3ClothDragger
{
public:
	b3ClothDragger(b3Ray3* ray, b3World* world) 
	{
		m_spring = false;
		m_ray = ray;
		m_world = world;
		m_cloth = nullptr;
	}

	~b3ClothDragger()
	{

	}

	bool IsDragging() const
	{
		return m_cloth != nullptr;
	}

	bool StartDragging()
	{
		B3_ASSERT(IsDragging() == false);

		b3RayCastSingleClothOutput rayOut;
		if (m_world->RayCastSingleCloth(&rayOut, m_ray->A(), m_ray->B()) == false)
		{
			return false;
		}

		m_cloth = rayOut.cloth;
		m_mesh = m_cloth->GetMesh();
		m_triangle = m_mesh->triangles + rayOut.triangle;
		m_x = rayOut.fraction;

		b3Particle* p1 = m_mesh->particles[m_triangle->v1];
		b3Particle* p2 = m_mesh->particles[m_triangle->v2];
		b3Particle* p3 = m_mesh->particles[m_triangle->v3];

		b3Vec3 v1 = p1->GetPosition();
		b3Vec3 v2 = p2->GetPosition();
		b3Vec3 v3 = p3->GetPosition();

		b3Vec3 B = GetPointB();

		float32 wABC[4];
		b3BarycentricCoordinates(wABC, v1, v2, v3, B);

		if (wABC[3] > B3_EPSILON)
		{
			m_u = wABC[0] / wABC[3];
			m_v = wABC[1] / wABC[3];
		}
		else
		{
			m_u = m_v = 0.0f;
		}

		if (m_spring)
		{
			b3ParticleDef pd;
			pd.type = e_staticParticle;
			pd.position = B;

			m_particle = m_cloth->CreateParticle(pd);

			{
				b3SpringForceDef sfd;
				sfd.p1 = m_particle;
				sfd.p2 = p1;
				sfd.restLength = 0.0f;
				sfd.structural = 10000.0f;
				m_s1 = (b3SpringForce*)m_cloth->CreateForce(sfd);
			}

			{
				b3SpringForceDef sfd;
				sfd.p1 = m_particle;
				sfd.p2 = p2;
				sfd.restLength = 0.0f;
				sfd.structural = 10000.0f;
				m_s2 = (b3SpringForce*)m_cloth->CreateForce(sfd);
			}

			{
				b3SpringForceDef sfd;
				sfd.p1 = m_particle;
				sfd.p2 = p3;
				sfd.restLength = 0.0f;
				sfd.structural = 10000.0f;
				m_s3 = (b3SpringForce*)m_cloth->CreateForce(sfd);
			}
		}
		else
		{
			m_t1 = p1->GetType();
			p1->SetType(e_staticParticle);

			m_t2 = p2->GetType();
			p2->SetType(e_staticParticle);

			m_t3 = p3->GetType();
			p3->SetType(e_staticParticle);
		}

		return true;
	}

	void Drag()
	{
		B3_ASSERT(IsDragging() == true);

		b3Vec3 A = GetPointA();
		b3Vec3 B = GetPointB();

		b3Vec3 dx = B - A;

		if (m_spring)
		{
			m_particle->SetPosition(B);
		}
		else
		{
			b3Particle* p1 = m_mesh->particles[m_triangle->v1];
			p1->ApplyTranslation(dx);

			b3Particle* p2 = m_mesh->particles[m_triangle->v2];
			p2->ApplyTranslation(dx);

			b3Particle* p3 = m_mesh->particles[m_triangle->v3];
			p3->ApplyTranslation(dx);
		}
	}

	void StopDragging()
	{
		B3_ASSERT(IsDragging() == true);

		m_cloth = nullptr;

		if (m_spring)
		{
			m_cloth->DestroyForce(m_s1);
			m_cloth->DestroyForce(m_s2);
			m_cloth->DestroyForce(m_s3);
			m_cloth->DestroyParticle(m_particle);
		}
		else
		{
			b3Particle* p1 = m_mesh->particles[m_triangle->v1];
			p1->SetType(m_t1);

			b3Particle* p2 = m_mesh->particles[m_triangle->v2];
			p2->SetType(m_t2);

			b3Particle* p3 = m_mesh->particles[m_triangle->v3];
			p3->SetType(m_t3);
		}
	}

	b3Vec3 GetPointA() const
	{
		B3_ASSERT(IsDragging() == true);
		
		b3Vec3 A = m_mesh->vertices[m_triangle->v1];
		b3Vec3 B = m_mesh->vertices[m_triangle->v2];
		b3Vec3 C = m_mesh->vertices[m_triangle->v3];

		return m_u * A + m_v * B + (1.0f - m_u - m_v) * C;
	}

	b3Vec3 GetPointB() const
	{
		B3_ASSERT(IsDragging() == true);
		
		return (1.0f - m_x) * m_ray->A() + m_x * m_ray->B();
	}
private:
	b3Ray3* m_ray;
	float32 m_x;

	b3World* m_world;

	b3Cloth* m_cloth;
	b3ClothMesh* m_mesh;
	b3ClothMeshTriangle* m_triangle;
	float32 m_u, m_v;

	bool m_spring;

	b3Particle* m_particle;
	b3SpringForce* m_s1;
	b3SpringForce* m_s2;
	b3SpringForce* m_s3;

	b3ParticleType m_t1, m_t2, m_t3;
};

#endif