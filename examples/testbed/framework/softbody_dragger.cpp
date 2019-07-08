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
	m_tetrahedron = nullptr;
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

	m_mesh = m_body->GetMesh();
	m_tetrahedron = m_mesh->tetrahedrons + rayOut.tetrahedron;
	m_v1 = m_tetrahedron->v1;
	m_v2 = m_tetrahedron->v2;
	m_v3 = m_tetrahedron->v3;
	m_v4 = m_tetrahedron->v4;
	m_x = rayOut.fraction;

	b3SoftBodyNode* n1 = m_body->GetVertexNode(m_v1);
	b3SoftBodyNode* n2 = m_body->GetVertexNode(m_v2);
	b3SoftBodyNode* n3 = m_body->GetVertexNode(m_v3);
	b3SoftBodyNode* n4 = m_body->GetVertexNode(m_v4);

	b3Vec3 v1 = n1->GetPosition();
	b3Vec3 v2 = n2->GetPosition();
	b3Vec3 v3 = n3->GetPosition();
	b3Vec3 v4 = n4->GetPosition();

	b3Vec3 B = GetPointB();

	float32 wABCD[5];
	b3BarycentricCoordinates(wABCD, v1, v2, v3, v4, B);

	if (wABCD[4] > B3_EPSILON)
	{
		m_tu = wABCD[0] / wABCD[4];
		m_tv = wABCD[1] / wABCD[4];
		m_tw = wABCD[2] / wABCD[4];
		m_tx = wABCD[3] / wABCD[4];
	}
	else
	{
		m_tu = m_tv = m_tw = m_tx = 0.0f;
	}

	return true;
}

void b3SoftBodyDragger::Drag()
{
	B3_ASSERT(IsDragging() == true);

	b3Vec3 A = GetPointA();
	b3Vec3 B = GetPointB();

	b3Vec3 dx = B - A;

	const float32 k = 100.0f;

	b3Vec3 f = k * dx;

	b3Vec3 f1 = m_tu * f;
	b3Vec3 f2 = m_tv * f;
	b3Vec3 f3 = m_tw * f;
	b3Vec3 f4 = m_tx * f;

	m_body->GetVertexNode(m_v1)->ApplyForce(f1);
	m_body->GetVertexNode(m_v2)->ApplyForce(f2);
	m_body->GetVertexNode(m_v3)->ApplyForce(f3);
	m_body->GetVertexNode(m_v4)->ApplyForce(f4);
}

void b3SoftBodyDragger::StopDragging()
{
	B3_ASSERT(IsDragging() == true);
	
	m_tetrahedron = nullptr;
}

b3Vec3 b3SoftBodyDragger::GetPointA() const
{
	B3_ASSERT(IsDragging() == true);

	b3Vec3 A = m_body->GetVertexNode(m_v1)->GetPosition();
	b3Vec3 B = m_body->GetVertexNode(m_v2)->GetPosition();
	b3Vec3 C = m_body->GetVertexNode(m_v3)->GetPosition();
	b3Vec3 D = m_body->GetVertexNode(m_v4)->GetPosition();

	return m_tu * A + m_tv * B + m_tw * C + m_tx * D;
}

b3Vec3 b3SoftBodyDragger::GetPointB() const
{
	B3_ASSERT(IsDragging() == true);
	return (1.0f - m_x) * m_ray->A() + m_x * m_ray->B();
}