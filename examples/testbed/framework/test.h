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

#ifndef TEST_H
#define TEST_H

#include <glfw/glfw3.h>
#include <bounce/bounce.h>

#include <testbed/framework/draw.h>
#include <testbed/framework/view_model.h>

inline float32 RandomFloat(float32 a, float32 b)
{
	float32 x = float32(rand()) / float32(RAND_MAX);
	float32 diff = b - a;
	float32 r = x * diff;
	return a + r;
}

class RayCastListener : public b3RayCastListener
{
public:
	float32 ReportShape(b3Shape* shape, const b3Vec3& point, const b3Vec3& normal, float32 fraction)
	{
		hit.shape = shape;
		hit.point = point;
		hit.normal = normal;
		hit.fraction = fraction;
		return 1.0f;
	}

	b3RayCastSingleOutput hit;
};

class BodyDragger
{
public:
	BodyDragger(Ray3* ray, b3World* world)
	{
		m_ray = ray;
		m_world = world;
		m_shape = nullptr;
		m_mouseJoint = nullptr;
	}

	~BodyDragger()
	{

	}
	
	bool StartDragging()
	{
		B3_ASSERT(m_mouseJoint == nullptr);

		b3RayCastSingleOutput out;
		if (m_world->RayCastSingle(&out, m_ray->A(), m_ray->B()) == false)
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

	bool IsSelected() const
	{
		return m_mouseJoint != nullptr;
	}


	Ray3* GetRay() const
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
	Ray3* m_ray;
	float32 m_x;

	b3World* m_world;
	b3Shape* m_shape;
	b3Vec3 m_p;
	b3MouseJoint* m_mouseJoint;
};

class Test : public b3ContactListener
{
public:
	Test();
	virtual ~Test();

	virtual void Save() { }

	virtual void Step();

	virtual void MouseMove(const Ray3& pw);
	virtual void MouseLeftDown(const Ray3& pw);
	virtual void MouseLeftUp(const Ray3& pw);
	virtual void KeyDown(int button) { }
	virtual void KeyUp(int button) { }

	virtual void BeginDragging() { }
	virtual void EndDragging() { }

	void BeginContact(b3Contact* c) override { }
	void EndContact(b3Contact* c) override { }
	void PreSolve(b3Contact* c) override { }

	b3World m_world;

	Ray3 m_bodyRay;
	BodyDragger m_bodyDragger;

	b3BoxHull m_groundHull;
	b3GridMesh<50, 50> m_groundMesh;
};

struct TestEntry
{
	typedef Test* (*TestCreate)();
	const char* name;
	TestCreate create;
};

extern TestEntry g_tests[];
extern u32 g_testCount;

#endif