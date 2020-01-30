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

#include <testbed/framework/softbody_dragger.h>

b3SoftBodyDragger::b3SoftBodyDragger(b3Ray3* ray, b3SoftBody* body)
{
	m_ray = ray;
	m_body = body;
	m_isDragging = false;
}

b3SoftBodyDragger::~b3SoftBodyDragger()
{

}

bool b3SoftBodyDragger::StartDragging()
{
	B3_ASSERT(IsDragging() == false);

	b3SoftBodyRayCastSingleOutput rayOut;
	if (m_body->RayCastSingle(&rayOut, m_ray->A(), m_ray->B()) == false)
	{
		return false;
	}

	m_isDragging = true;
	m_x = rayOut.fraction;

	const b3SoftBodyMesh* mesh = m_body->GetMesh();
	const b3SoftBodyMeshTriangle* triangle = mesh->triangles + rayOut.triangle;

	m_n1 = m_body->GetNode(triangle->v1);
	m_n2 = m_body->GetNode(triangle->v2);
	m_n3 = m_body->GetNode(triangle->v3);

	b3Vec3 v1 = m_n1->GetPosition();
	b3Vec3 v2 = m_n2->GetPosition();
	b3Vec3 v3 = m_n3->GetPosition();

	b3Vec3 B = GetPointB();

	scalar wABC[4];
	b3BarycentricCoordinates(wABC, v1, v2, v3, B);

	if (wABC[3] > B3_EPSILON)
	{
		m_tu = wABC[0] / wABC[3];
		m_tv = wABC[1] / wABC[3];
		m_tw = wABC[2] / wABC[3];
	}
	else
	{
		m_tu = m_tv = m_tw = 0.0f;
	}

	m_t1 = m_n1->GetType();
	m_n1->SetType(e_staticSoftBodyNode);

	m_t2 = m_n2->GetType();
	m_n2->SetType(e_staticSoftBodyNode);

	m_t3 = m_n3->GetType();
	m_n3->SetType(e_staticSoftBodyNode);

	return true;
}

void b3SoftBodyDragger::Drag()
{
	B3_ASSERT(IsDragging() == true);

	b3Vec3 A = GetPointA();
	b3Vec3 B = GetPointB();

	b3Vec3 dx = B - A;

	m_n1->ApplyTranslation(dx);
	m_n2->ApplyTranslation(dx);
	m_n3->ApplyTranslation(dx);
}

void b3SoftBodyDragger::StopDragging()
{
	B3_ASSERT(IsDragging() == true);
	
	m_n1->SetType(m_t1);
	m_n2->SetType(m_t2);
	m_n3->SetType(m_t3);
	
	m_isDragging = false;
}

b3Vec3 b3SoftBodyDragger::GetPointA() const
{
	B3_ASSERT(IsDragging() == true);

	b3Vec3 A = m_n1->GetPosition() + m_n1->GetTranslation();
	b3Vec3 B = m_n2->GetPosition() + m_n2->GetTranslation();
	b3Vec3 C = m_n3->GetPosition() + m_n3->GetTranslation();

	return m_tu * A + m_tv * B + m_tw * C;
}

b3Vec3 b3SoftBodyDragger::GetPointB() const
{
	B3_ASSERT(IsDragging() == true);
	return (1.0f - m_x) * m_ray->A() + m_x * m_ray->B();
}