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

#ifndef CLOTH_TESH_H
#define CLOTH_TESH_H

class ClothDragger
{
public:
	ClothDragger(Ray3* ray, b3Cloth*& cloth) : m_ray(ray), m_cloth(cloth)
	{
		m_isSelected = false;
		m_spring = false;
	}

	~ClothDragger()
	{

	}

	bool IsSelected() const
	{
		return m_isSelected;
	}

	b3Vec3 GetPointA() const
	{
		B3_ASSERT(m_isSelected);

		b3ClothMesh* m = m_cloth->GetMesh();
		b3ClothMeshTriangle* t = m->triangles + m_selection;

		b3Vec3 A = m->vertices[t->v1];
		b3Vec3 B = m->vertices[t->v2];
		b3Vec3 C = m->vertices[t->v3];

		return m_u * A + m_v * B + (1.0f - m_u - m_v) * C;
	}

	b3Vec3 GetPointB() const
	{
		B3_ASSERT(m_isSelected);
		return (1.0f - m_x) * m_ray->A() + m_x * m_ray->B();
	}

	bool StartDragging()
	{
		B3_ASSERT(m_isSelected == false);

		b3RayCastInput rayIn;
		rayIn.p1 = m_ray->A();
		rayIn.p2 = m_ray->B();
		rayIn.maxFraction = B3_MAX_FLOAT;

		b3ClothRayCastOutput rayOut;
		if (m_cloth->RayCast(&rayOut, &rayIn) == false)
		{
			return false;
		}

		m_isSelected = true;
		
		m_selection = rayOut.triangle;
		m_x = rayOut.fraction;

		b3ClothMesh* m = m_cloth->GetMesh();
		b3ClothMeshTriangle* t = m->triangles + m_selection;

		b3Particle* p1 = m->particles[t->v1];
		b3Particle* p2 = m->particles[t->v2];
		b3Particle* p3 = m->particles[t->v3];

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
		B3_ASSERT(m_isSelected);

		b3ClothMesh* m = m_cloth->GetMesh();
		b3ClothMeshTriangle* t = m->triangles + m_selection;

		b3Vec3 A = GetPointA();
		b3Vec3 B = GetPointB();

		b3Vec3 dx = B - A;

		if (m_spring)
		{
			m_particle->ApplyTranslation(dx);
		}
		else
		{
			b3Particle* p1 = m->particles[t->v1];
			p1->ApplyTranslation(dx);

			b3Particle* p2 = m->particles[t->v2];
			p2->ApplyTranslation(dx);

			b3Particle* p3 = m->particles[t->v3];
			p3->ApplyTranslation(dx);
		}
	}

	void StopDragging()
	{
		B3_ASSERT(m_isSelected);

		m_isSelected = false;

		if (m_spring)
		{
			m_cloth->DestroyForce(m_s1);
			m_cloth->DestroyForce(m_s2);
			m_cloth->DestroyForce(m_s3);
			m_cloth->DestroyParticle(m_particle);
		}
		else
		{
			b3ClothMesh* m = m_cloth->GetMesh();
			b3ClothMeshTriangle* t = m->triangles + m_selection;

			b3Particle* p1 = m->particles[t->v1];
			p1->SetType(m_t1);

			b3Particle* p2 = m->particles[t->v2];
			p2->SetType(m_t2);

			b3Particle* p3 = m->particles[t->v3];
			p3->SetType(m_t3);
		}
	}

private:
	bool m_isSelected;

	Ray3* m_ray;
	float32 m_x;

	b3Cloth*& m_cloth;
	u32 m_selection;
	float32 m_u, m_v;
	
	bool m_spring;

	b3Particle* m_particle;
	b3SpringForce* m_s1;
	b3SpringForce* m_s2;
	b3SpringForce* m_s3;

	b3ParticleType m_t1, m_t2, m_t3;
};

class ClothTest : public Test
{
public:
	ClothTest() : m_clothDragger(&m_clothRay, m_cloth)
	{
		m_world.SetGravity(b3Vec3(0.0f, -10.0f, 0.0f));

		m_clothRay.origin.SetZero();
		m_clothRay.direction.Set(0.0f, 0.0f, -1.0f);
		m_clothRay.fraction = g_camera->m_zFar;

		m_cloth = nullptr;
	}

	void Step()
	{
		Test::Step();

		m_cloth->Apply();

		m_cloth->Draw();

		extern u32 b3_clothSolverIterations;
		g_draw->DrawString(b3Color_white, "Iterations = %u", b3_clothSolverIterations);
		
		float32 E = m_cloth->GetEnergy();
		g_draw->DrawString(b3Color_white, "E = %f", E);

		if (m_clothDragger.IsSelected() == true)
		{
			g_draw->DrawSegment(m_clothDragger.GetPointA(), m_clothDragger.GetPointB(), b3Color_white);
		}
	}

	void MouseMove(const Ray3& pw)
	{
		m_clothRay = pw;

		if (m_clothDragger.IsSelected() == true)
		{
			m_clothDragger.Drag();
		}
	}

	void MouseLeftDown(const Ray3& pw)
	{
		if (m_clothDragger.IsSelected() == false)
		{
			m_clothDragger.StartDragging();
		}
	}

	void MouseLeftUp(const Ray3& pw)
	{
		if (m_clothDragger.IsSelected() == true)
		{
			m_clothDragger.StopDragging();
		}
	}

	Ray3 m_clothRay;
	b3Cloth* m_cloth;
	ClothDragger m_clothDragger;
};

#endif