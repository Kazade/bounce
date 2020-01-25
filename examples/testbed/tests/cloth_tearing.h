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

#ifndef CLOTH_TEARING_H
#define CLOTH_TEARING_H

class ClothTearing : public Test
{
public:
	enum
	{
		e_w = 10,
		e_h = 10
	};

	ClothTearing()
	{
		g_camera->m_zoom = 20.0f;

		b3GridClothMesh<e_w, e_h> m;

		b3ClothParticle** particles = (b3ClothParticle * *)b3Alloc(m.vertexCount * sizeof(b3ClothParticle*));
		for (u32 i = 0; i < m.vertexCount; ++i)
		{
			b3ClothParticleDef pd;
			pd.type = e_dynamicClothParticle;
			pd.position = m.vertices[i];

			b3ClothParticle* p = m_cloth.CreateParticle(pd);
			particles[i] = p;

			b3ClothSphereShapeDef sd;
			sd.p = p;
			sd.radius = 0.2f;
			sd.friction = 0.4f;

			m_cloth.CreateSphereShape(sd);
		}

		for (u32 i = 0; i < m.triangleCount; ++i)
		{
			u32 v1 = m.triangles[i].v1;
			u32 v2 = m.triangles[i].v2;
			u32 v3 = m.triangles[i].v3;

			b3ClothParticle* p1 = particles[v1];
			b3ClothParticle* p2 = particles[v2];
			b3ClothParticle* p3 = particles[v3];

			b3ClothTriangleShapeDef tsd;
			tsd.p1 = p1;
			tsd.p2 = p2;
			tsd.p3 = p3;
			tsd.v1 = m.vertices[v1];
			tsd.v2 = m.vertices[v2];
			tsd.v3 = m.vertices[v3];
			tsd.density = 0.1f;

			m_cloth.CreateTriangleShape(tsd);

			{
				b3SpringForceDef sfd;
				sfd.Initialize(p1, p2, 1000.0f, 10.0f);

				CreateSpringForce(sfd);
			}

			{
				b3SpringForceDef sfd;
				sfd.Initialize(p2, p3, 1000.0f, 10.0f);

				CreateSpringForce(sfd);
			}

			{
				b3SpringForceDef sfd;
				sfd.Initialize(p3, p1, 1000.0f, 10.0f);

				CreateSpringForce(sfd);
			}
		}

		for (u32 i = 0; i < e_w + 1; ++i)
		{
			u32 vertex = m.GetVertex(0, i);
			particles[vertex]->SetType(e_staticClothParticle);
		}

		b3Free(particles);

		m_cloth.SetGravity(b3Vec3(0.0f, -9.8f, 0.0f));

		m_clothDragger = new b3ClothDragger(&m_ray, &m_cloth);
		m_clothDragger->SetStaticDrag(false);
	}

	~ClothTearing()
	{
		delete m_clothDragger;
	}

	b3SpringForce* FindSpringForce(b3ClothParticle* p1, b3ClothParticle* p2)
	{
		for (b3Force* f = m_cloth.GetForceList().m_head; f; f = f->GetNext())
		{
			if (f->GetType() != e_springForce)
			{
				continue;
			}

			b3SpringForce* sf = (b3SpringForce*)f;

			b3ClothParticle* sp1 = sf->GetParticle1();
			b3ClothParticle* sp2 = sf->GetParticle2();

			if (sp1 == p1 && sp2 == p2)
			{
				return sf;
			}

			if (sp1 == p2 && sp2 == p1)
			{
				return sf;
			}
		}

		return nullptr;
	}

	b3SpringForce* CreateSpringForce(const b3SpringForceDef& def)
	{
		b3SpringForce* sf = FindSpringForce(def.p1, def.p2);
		if (sf != nullptr)
		{
			return sf;
		}

		return (b3SpringForce*)m_cloth.CreateForce(def);
	}

	void DrawSpringForces()
	{
		for (b3Force* f = m_cloth.GetForceList().m_head; f; f = f->GetNext())
		{
			if (f->GetType() != e_springForce)
			{
				continue;
			}

			b3SpringForce* s = (b3SpringForce*)f;

			b3ClothParticle* p1 = s->GetParticle1();
			b3ClothParticle* p2 = s->GetParticle2();

			g_draw->DrawSegment(p1->GetPosition(), p2->GetPosition(), b3Color_black);
		}
	}

	void Partition(b3ClothParticle* p, const b3Plane& plane,
		b3Array<b3ClothTriangleShape*>& above,
		b3Array<b3ClothTriangleShape*>& below)
	{
		for (b3ClothTriangleShape* t = m_cloth.GetTriangleShapeList().m_head; t; t = t->GetNext())
		{
			b3ClothParticle* p1 = t->GetParticle1();
			b3ClothParticle* p2 = t->GetParticle2();
			b3ClothParticle* p3 = t->GetParticle3();

			if (p1 != p && p2 != p && p3 != p)
			{
				continue;
			}

			b3Vec3 x1 = p1->GetPosition();
			b3Vec3 x2 = p2->GetPosition();
			b3Vec3 x3 = p3->GetPosition();

			b3Vec3 center = (x1 + x2 + x3) / 3.0f;

			scalar distance = b3Distance(center, plane);
			if (distance > 0.0f)
			{
				above.PushBack(t);
			}
			else
			{
				below.PushBack(t);
			}
		}
	}

	bool HasSpring(const b3Array<b3ClothTriangleShape*>& triangles,
		b3ClothParticle* pOld, b3ClothParticle* pOther)
	{
		for (u32 i = 0; i < triangles.Count(); ++i)
		{
			b3ClothTriangleShape* triangle = triangles[i];

			b3ClothParticle* tp1 = triangle->GetParticle1();
			b3ClothParticle* tp2 = triangle->GetParticle2();
			b3ClothParticle* tp3 = triangle->GetParticle3();

			// 1, 2
			if (tp1 == pOld && tp2 == pOther)
			{
				return true;
			}

			// 2, 1
			if (tp2 == pOld && tp1 == pOther)
			{
				return true;
			}

			// 2, 3
			if (tp2 == pOld && tp3 == pOther)
			{
				return true;
			}

			// 3, 2
			if (tp3 == pOld && tp2 == pOther)
			{
				return true;
			}

			// 3, 1
			if (tp3 == pOld && tp1 == pOther)
			{
				return true;
			}

			// 1, 3
			if (tp1 == pOld && tp3 == pOther)
			{
				return true;
			}
		}

		return false;
	}

	bool SplitParticle(b3ClothParticle* pOld, const b3Plane& plane)
	{
		// Collect triangles.
		b3StackArray<b3ClothTriangleShape*, 32> trianglesAbove, trianglesBelow;
		Partition(pOld, plane, trianglesAbove, trianglesBelow);

		// There must be at least one triangle on each side of the plane.
		if (trianglesAbove.Count() == 0 || trianglesBelow.Count() == 0)
		{
			return false;
		}

		b3ClothParticleDef pdNew;
		pdNew.type = pOld->GetType();
		pdNew.position = pOld->GetPosition() - 0.2f * plane.normal;

		b3ClothParticle* pNew = m_cloth.CreateParticle(pdNew);

		b3ClothSphereShapeDef ssdNew;
		ssdNew.p = pNew;
		ssdNew.radius = 0.2f;
		ssdNew.friction = 0.4f;
		
		m_cloth.CreateSphereShape(ssdNew);

		for (u32 i = 0; i < trianglesBelow.Count(); ++i)
		{
			b3ClothTriangleShape* triangle = trianglesBelow[i];

			b3ClothParticle* p1 = triangle->GetParticle1();
			b3ClothParticle* p2 = triangle->GetParticle2();
			b3ClothParticle* p3 = triangle->GetParticle3();

			m_cloth.DestroyTriangleShape(triangle);

			if (p1 == pOld)
			{
				b3ClothTriangleShapeDef tdNew;
				tdNew.p1 = pNew;
				tdNew.p2 = p2;
				tdNew.p3 = p3;
				tdNew.v1 = pNew->GetPosition();
				tdNew.v2 = p2->GetPosition();
				tdNew.v3 = p3->GetPosition();

				m_cloth.CreateTriangleShape(tdNew);

				b3SpringForce* sf1 = FindSpringForce(p1, p2);
				if (sf1)
				{
					b3SpringForceDef sNew;
					sNew.p1 = pNew;
					sNew.p2 = p2;
					sNew.restLength = sf1->GetRestLenght();
					sNew.structural = sf1->GetStructuralStiffness();
					sNew.damping = sf1->GetDampingStiffness();

					m_cloth.CreateForce(sNew);

					if (HasSpring(trianglesAbove, p1, p2) == false)
					{
						m_cloth.DestroyForce(sf1);
					}
				}

				b3SpringForce* sf2 = FindSpringForce(p3, p1);
				if (sf2)
				{
					b3SpringForceDef sNew;
					sNew.p1 = p3;
					sNew.p2 = pNew;
					sNew.restLength = sf2->GetRestLenght();
					sNew.structural = sf2->GetStructuralStiffness();
					sNew.damping = sf2->GetDampingStiffness();
					
					m_cloth.CreateForce(sNew);
					
					if (HasSpring(trianglesAbove, p3, p1) == false)
					{
						m_cloth.DestroyForce(sf2);
					}
				}
			}
			
			if (p2 == pOld)
			{
				b3ClothTriangleShapeDef tdNew;
				tdNew.p1 = p1;
				tdNew.p2 = pNew;
				tdNew.p3 = p3;
				tdNew.v1 = p1->GetPosition();
				tdNew.v2 = pNew->GetPosition();
				tdNew.v3 = p3->GetPosition();

				m_cloth.CreateTriangleShape(tdNew);

				b3SpringForce* sf1 = FindSpringForce(p1, p2);
				if (sf1)
				{
					b3SpringForceDef sNew;
					sNew.p1 = p1;
					sNew.p2 = pNew;
					sNew.restLength = sf1->GetRestLenght();
					sNew.structural = sf1->GetStructuralStiffness();
					sNew.damping = sf1->GetDampingStiffness();

					m_cloth.CreateForce(sNew);
					
					if (HasSpring(trianglesAbove, p1, p2) == false)
					{
						m_cloth.DestroyForce(sf1);
					}
				}

				b3SpringForce* sf2 = FindSpringForce(p2, p3);
				if (sf2)
				{
					b3SpringForceDef sNew;
					sNew.p1 = pNew;
					sNew.p2 = p3;
					sNew.restLength = sf2->GetRestLenght();
					sNew.structural = sf2->GetStructuralStiffness();
					sNew.damping = sf2->GetDampingStiffness();
					
					m_cloth.CreateForce(sNew);
					
					if (HasSpring(trianglesAbove, p2, p3) == false)
					{
						m_cloth.DestroyForce(sf2);
					}
				}
			}
			
			if (p3 == pOld)
			{
				b3ClothTriangleShapeDef tdNew;
				tdNew.p1 = p1;
				tdNew.p2 = p2;
				tdNew.p3 = pNew;
				tdNew.v1 = p1->GetPosition();
				tdNew.v2 = p2->GetPosition();
				tdNew.v3 = pNew->GetPosition();

				m_cloth.CreateTriangleShape(tdNew);
				
				b3SpringForce* sf1 = FindSpringForce(p2, p3);
				if (sf1)
				{
					b3SpringForceDef sNew;
					sNew.p1 = p2;
					sNew.p2 = pNew;
					sNew.restLength = sf1->GetRestLenght();
					sNew.structural = sf1->GetStructuralStiffness();
					sNew.damping = sf1->GetDampingStiffness();
					
					m_cloth.CreateForce(sNew);
					
					if (HasSpring(trianglesAbove, p2, p3) == false)
					{
						m_cloth.DestroyForce(sf1);
					}
				}

				b3SpringForce* sf2 = FindSpringForce(p3, p1);
				if (sf2)
				{
					b3SpringForceDef sNew;
					sNew.p1 = pNew;
					sNew.p2 = p1;
					sNew.restLength = sf2->GetRestLenght();
					sNew.structural = sf2->GetStructuralStiffness();
					sNew.damping = sf2->GetDampingStiffness();
					
					m_cloth.CreateForce(sNew);
					
					if (HasSpring(trianglesAbove, p3, p1) == false)
					{
						m_cloth.DestroyForce(sf2);
					}
				}
			}
		}

		return true;
	}

	bool Tear()
	{
		b3Force* f = m_cloth.GetForceList().m_head;
		while (f)
		{
			if (f->GetType() != e_springForce)
			{
				f = f->GetNext();
				continue;
			}

			b3SpringForce* s = (b3SpringForce*)f;
			f = f->GetNext();

			b3Vec3 tension = s->GetActionForce();

			const scalar kMaxTension = 1000.0f;

			if (b3LengthSquared(tension) <= kMaxTension * kMaxTension)
			{
				continue;
			}

			b3ClothParticle* p1 = s->GetParticle1();
			b3ClothParticle* p2 = s->GetParticle2();

			b3Vec3 x1 = p1->GetPosition();
			b3Vec3 x2 = p2->GetPosition();

			if (p1->GetType() == e_dynamicClothParticle)
			{
				b3Vec3 n = b3Normalize(x2 - x1);
				b3Plane plane(n, x1);

				bool wasSplit = SplitParticle(p1, plane);
				if (wasSplit)
				{
					return true;
				}
			}

			if (p2->GetType() == e_dynamicClothParticle)
			{
				b3Vec3 n = b3Normalize(x1 - x2);
				b3Plane plane(n, x2);

				bool wasSplit = SplitParticle(p2, plane);
				if (wasSplit)
				{
					return true;
				}
			}
		}

		return false;
	}

	void Step()
	{
		Test::Step();

		m_cloth.Step(g_testSettings->inv_hertz, g_testSettings->velocityIterations, g_testSettings->positionIterations);

		while (Tear());

		m_cloth.Draw();

		DrawSpringForces();

		if (m_clothDragger->IsDragging())
		{
			b3Vec3 pA = m_clothDragger->GetPointA();
			b3Vec3 pB = m_clothDragger->GetPointB();

			g_draw->DrawPoint(pA, 4.0f, b3Color_green);

			g_draw->DrawPoint(pB, 4.0f, b3Color_green);

			g_draw->DrawSegment(pA, pB, b3Color_white);
		}

		extern u32 b3_clothSolverIterations;
		g_draw->DrawString(b3Color_white, "Iterations = %d", b3_clothSolverIterations);

		scalar E = m_cloth.GetEnergy();
		g_draw->DrawString(b3Color_white, "E = %f", E);
	}

	void MouseMove(const b3Ray3& pw)
	{
		Test::MouseMove(pw);

		if (m_clothDragger->IsDragging() == true)
		{
			m_clothDragger->Drag();
		}
	}

	void MouseLeftDown(const b3Ray3& pw)
	{
		Test::MouseLeftDown(pw);

		if (m_clothDragger->IsDragging() == false)
		{
			m_clothDragger->StartDragging();
		}
	}

	void MouseLeftUp(const b3Ray3& pw)
	{
		Test::MouseLeftUp(pw);

		if (m_clothDragger->IsDragging() == true)
		{
			m_clothDragger->StopDragging();
		}
	}

	static Test* Create()
	{
		return new ClothTearing();
	}

	b3Cloth m_cloth;
	b3ClothDragger* m_clothDragger;
};

#endif