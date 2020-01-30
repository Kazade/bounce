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

#include <testbed/framework/body_dragger.h>

b3BodyDragger::b3BodyDragger(b3Ray3* ray, b3World* world)
{
	m_ray = ray;
	m_world = world;
	m_shape = nullptr;
	m_mouseJoint = nullptr;
}

b3BodyDragger::~b3BodyDragger()
{

}

bool b3BodyDragger::StartDragging()
{
	B3_ASSERT(IsDragging() == false);

	class RayCastFilter : public b3RayCastFilter
	{
	public:
		bool ShouldRayCast(b3Shape* shape)
		{
			return true;
		}
	};

	RayCastFilter filter;

	b3RayCastSingleOutput out;
	if (m_world->RayCastSingle(&out, &filter, m_ray->A(), m_ray->B()) == false)
	{
		return false;
	}
	
	m_x = out.fraction;
	m_shape = out.shape;

	b3BodyDef bd;
	b3Body* groundBody = m_world->CreateBody(bd);

	b3Body* body = m_shape->GetBody();
	body->SetAwake(true);

	b3MouseJointDef jd;
	jd.bodyA = groundBody;
	jd.bodyB = body;
	jd.target = out.point;
	jd.maxForce = 1000.0f * body->GetMass();

	m_mouseJoint = (b3MouseJoint*)m_world->CreateJoint(jd);

	m_p = body->GetLocalPoint(out.point);

	return true;
}

void b3BodyDragger::Drag()
{
	B3_ASSERT(IsDragging() == true);
	m_mouseJoint->SetTarget(GetPointB());
}

void b3BodyDragger::StopDragging()
{
	B3_ASSERT(IsDragging() == true);
	b3Body* groundBody = m_mouseJoint->GetBodyA();
	m_world->DestroyJoint(m_mouseJoint);
	m_mouseJoint = nullptr;
	m_world->DestroyBody(groundBody);
	m_shape = nullptr;
}

b3Shape* b3BodyDragger::GetShape() const
{
	B3_ASSERT(IsDragging() == true);
	return m_shape;
}

b3Vec3 b3BodyDragger::GetPointA() const
{
	B3_ASSERT(IsDragging() == true);
	return m_shape->GetBody()->GetWorldPoint(m_p);
}

b3Vec3 b3BodyDragger::GetPointB() const
{
	B3_ASSERT(IsDragging() == true);
	return (1.0f - m_x) * m_ray->A() + m_x * m_ray->B();
}