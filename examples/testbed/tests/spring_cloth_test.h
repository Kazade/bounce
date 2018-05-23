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

#ifndef SPRING_CLOTH_TESH_H
#define SPRING_CLOTH_TESH_H

class ClothDragger
{
public:
	ClothDragger(Ray3* ray, b3SpringCloth* cloth)
	{
		m_ray = ray;
		m_cloth = cloth;
		m_isSelected = false;
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

		if (Select(m_selection, m_x) == false)
		{
			return false;
		}

		m_isSelected = true;

		b3ClothMesh* m = m_cloth->GetMesh();
		b3ClothMeshTriangle* t = m->triangles + m_selection;

		m_t1 = m_cloth->GetType(t->v1);
		m_cloth->SetType(t->v1, b3MassType::e_staticMass);

		m_t2 = m_cloth->GetType(t->v2);
		m_cloth->SetType(t->v2, b3MassType::e_staticMass);
		
		m_t3 = m_cloth->GetType(t->v3);
		m_cloth->SetType(t->v3, b3MassType::e_staticMass);

		b3Vec3 v1 = m_cloth->GetPosition(t->v1);
		b3Vec3 v2 = m_cloth->GetPosition(t->v2);
		b3Vec3 v3 = m_cloth->GetPosition(t->v3);

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

		b3Vec3 v1 = m_cloth->GetPosition(t->v1);
		v1 += dx;
		m_cloth->SetPosition(t->v1, v1);

		b3Vec3 v2 = m_cloth->GetPosition(t->v2);
		v2 += dx;
		m_cloth->SetPosition(t->v2, v2);

		b3Vec3 v3 = m_cloth->GetPosition(t->v3);
		v3 += dx;
		m_cloth->SetPosition(t->v3, v3);
	}

	void StopDragging()
	{
		B3_ASSERT(m_isSelected);

		m_isSelected = false;

		b3ClothMesh* m = m_cloth->GetMesh();
		b3ClothMeshTriangle* t = m->triangles + m_selection;

		m_cloth->SetType(t->v1, m_t1);
		m_cloth->SetType(t->v2, m_t2);
		m_cloth->SetType(t->v3, m_t3);
	}

private:
	bool RayCast(b3RayCastOutput* output, u32 triangleIndex) const
	{
		b3ClothMesh* mesh = m_cloth->GetMesh();		
		B3_ASSERT(triangleIndex < mesh->triangleCount);
		b3ClothMeshTriangle* triangle = mesh->triangles + triangleIndex;
		
		b3Vec3 v1 = mesh->vertices[triangle->v1];
		b3Vec3 v2 = mesh->vertices[triangle->v2];
		b3Vec3 v3 = mesh->vertices[triangle->v3];

		b3Vec3 p1 = m_ray->A();
		b3Vec3 p2 = m_ray->B();
		b3Vec3 d = p2 - p1;
		float32 maxFraction = b3Length(d);

		b3Vec3 n = b3Cross(v2 - v1, v3 - v1);
		n.Normalize();

		float32 numerator = b3Dot(n, v1 - p1);
		float32 denominator = b3Dot(n, d);

		if (denominator == 0.0f)
		{
			return false;
		}

		float32 t = numerator / denominator;

		// Is the intersection not on the segment?
		if (t < 0.0f || maxFraction < t)
		{
			return false;
		}

		b3Vec3 q = p1 + t * d;

		// Barycentric coordinates for q
		b3Vec3 Q = q;
		b3Vec3 A = v1;
		b3Vec3 B = v2;
		b3Vec3 C = v3;

		b3Vec3 AB = B - A;
		b3Vec3 AC = C - A;

		b3Vec3 QA = A - Q;
		b3Vec3 QB = B - Q;
		b3Vec3 QC = C - Q;

		b3Vec3 QB_x_QC = b3Cross(QB, QC);
		b3Vec3 QC_x_QA = b3Cross(QC, QA);
		b3Vec3 QA_x_QB = b3Cross(QA, QB);

		b3Vec3 AB_x_AC = b3Cross(AB, AC);

		float32 u = b3Dot(QB_x_QC, AB_x_AC);
		float32 v = b3Dot(QC_x_QA, AB_x_AC);
		float32 w = b3Dot(QA_x_QB, AB_x_AC);

		// This tolerance helps intersections lying on  
		// shared edges to not be missed.
		const float32 kTol = -0.005f;

		// Is the intersection on the triangle?
		if (u > kTol && v > kTol && w > kTol)
		{
			output->fraction = t;

			// Does the ray start from below or above the triangle?
			if (numerator > 0.0f)
			{
				output->normal = -n;
			}
			else
			{
				output->normal = n;
			}

			return true;
		}

		return false;
	}

	bool Select(u32& selection, float32& fraction) const
	{
		b3ClothMesh* m = m_cloth->GetMesh();

		b3RayCastInput input;
		input.p1 = m_ray->A();
		input.p2 = m_ray->B();
		input.maxFraction = m_ray->fraction;

		float32 minFraction = B3_MAX_FLOAT;
		b3Vec3 minNormal(0.0f, 0.0f, 0.0f);
		u32 minIndex = ~0;

		for (u32 i = 0; i < m->triangleCount; ++i)
		{
			b3RayCastOutput subOutput;
			if (RayCast(&subOutput, i) == true)
			{
				if (subOutput.fraction < minFraction)
				{
					minFraction = subOutput.fraction;
					minNormal = subOutput.normal;
					minIndex = i;
				}
			}
		}

		if (minIndex != ~0)
		{
			selection = minIndex;
			fraction = minFraction;
			return true;
		}

		return false;
	}

	bool m_isSelected;

	Ray3* m_ray;
	float32 m_x;

	b3SpringCloth * m_cloth;
	u32 m_selection;
	float32 m_u, m_v;
	b3MassType m_t1, m_t2, m_t3;
};

class SpringClothTest : public Test
{
public:
	SpringClothTest() : m_clothDragger(&m_clothRay, &m_cloth)
	{
		m_clothRay.origin.SetZero();
		m_clothRay.direction.Set(0.0f, 0.0f, -1.0f);
		m_clothRay.fraction = g_camera->m_zFar;
	}

	void Step()
	{
		float32 dt = g_testSettings->inv_hertz;

		m_cloth.Step(dt);
		m_cloth.Apply();

		b3Shape** shapes = m_cloth.GetShapes();
		for (u32 i = 0; i < m_cloth.GetShapeCount(); ++i)
		{
			b3Shape* s = shapes[i];

			b3Transform xf;
			xf.SetIdentity();

			g_draw->DrawSolidShape(s, b3Color_white, xf);
		}

		m_cloth.Draw();

		b3SpringClothStep step = m_cloth.GetStep();

		g_draw->DrawString(b3Color_white, "Iterations = %u", step.iterations);
		
		float32 E = m_cloth.GetEnergy();
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

	b3StackAllocator m_clothAllocator;
	b3SpringCloth m_cloth;
	
	ClothDragger m_clothDragger;
};

#endif