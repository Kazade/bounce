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

#ifndef B3_BODY_DRAGGER_H
#define B3_BODY_DRAGGER_H

#include <bounce/common/geometry.h>
#include <bounce/dynamics/body.h>

// A body dragger.
class b3BodyDragger
{
public:
	b3BodyDragger(b3Ray3* ray, b3World* world)
	{
		m_ray = ray;
		m_world = world;
		m_shape = nullptr;
		m_mouseJoint = nullptr;
	}

	~b3BodyDragger()
	{

	}

	bool IsDragging() const
	{
		return m_shape != nullptr;
	}

	bool StartDragging()
	{
		B3_ASSERT(m_mouseJoint == nullptr);

		b3ShapeRayCastSingleOutput out;
		if (m_world->RayCastSingleShape(&out, m_ray->A(), m_ray->B()) == false)
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
		jd.maxForce = 2000.0f * body->GetMass();

		m_mouseJoint = (b3MouseJoint*)m_world->CreateJoint(jd);

		m_p = body->GetLocalPoint(out.point);

		return true;
	}

	void Drag()
	{
		B3_ASSERT(m_mouseJoint);
		m_mouseJoint->SetTarget(GetPointB());
	}

	void StopDragging()
	{
		B3_ASSERT(m_mouseJoint);

		b3Body* groundBody = m_mouseJoint->GetBodyA();
		m_world->DestroyJoint(m_mouseJoint);
		m_mouseJoint = nullptr;
		m_world->DestroyBody(groundBody);
		m_shape = nullptr;
	}

	b3Ray3* GetRay() const
	{
		return m_ray;
	}

	b3Body* GetBody() const
	{
		B3_ASSERT(m_shape);
		return m_shape->GetBody();
	}

	b3Vec3 GetPointA() const
	{
		B3_ASSERT(m_shape);
		return m_shape->GetBody()->GetWorldPoint(m_p);
	}

	b3Vec3 GetPointB() const
	{
		B3_ASSERT(m_mouseJoint);
		return (1.0f - m_x) * m_ray->A() + m_x * m_ray->B();
	}

private:
	b3Ray3 * m_ray;
	float32 m_x;

	b3World* m_world;
	b3Shape* m_shape;
	b3Vec3 m_p;
	b3MouseJoint* m_mouseJoint;
};


#endif