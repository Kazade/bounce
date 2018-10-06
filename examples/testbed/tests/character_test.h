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

#ifndef CHARACTER_TEST_H
#define CHARACTER_TEST_H

struct Plane
{
	b3Vec3 n;
	b3Vec3 p;
};

class CharacterController
{
public:
	CharacterController(b3SphereShape* shape)
	{
		m_characterShape = shape;
		m_characterBody = m_characterShape->GetBody();
		m_world = m_characterBody->GetWorld();
		m_translation.SetZero();
		m_isGrounded = false;
		
		UpdateGrounded();
	}

	~CharacterController()
	{

	}

	bool IsGrounded() const
	{
		return m_isGrounded;
	}

	void Move(const b3Vec3& translation)
	{
		m_translation += translation;
	}

	void Solve()
	{
		b3StackArray<b3Shape*, 32> shapes;
		CollectStaticShapes(shapes);

		b3StackArray<Plane, 32> planes;
		CollectOverlapPlanes(planes, shapes);

		b3Vec3 startPosition = m_characterBody->GetWorldCenter();

		b3Vec3 targetPosition = startPosition + m_translation;

		m_translation.SetZero();

		b3Vec3 solvePosition = SolvePositionConstraints(planes, targetPosition);

		b3Vec3 oldSolvePosition = solvePosition;

		for (;;)
		{
			b3Vec3 translation = solvePosition - startPosition;

			CollectSweepPlanes(planes, shapes, translation);

			solvePosition = SolvePositionConstraints(planes, targetPosition);

			const float32 tolerance = 0.05f;

			if (b3DistanceSquared(oldSolvePosition, solvePosition) < tolerance * tolerance)
			{
				break;
			}

			oldSolvePosition = solvePosition;
		}

		// Update body
		b3Quat orientation = m_characterBody->GetOrientation();

		b3Vec3 axis;
		float32 angle;
		orientation.GetAxisAngle(&axis, &angle);

		m_characterBody->SetTransform(solvePosition, axis, angle);
	}

	void Step()
	{
		Solve();

		UpdateGrounded();
	}
private:
	b3Sphere GetCharacterSphere() const
	{
		b3Sphere sphere;
		sphere.vertex = m_characterBody->GetWorldPoint(m_characterShape->m_center);
		sphere.radius = m_characterShape->m_radius;
		return sphere;
	}

	void UpdateGrounded()
	{
		b3StackArray<b3Shape*, 32> shapes;
		CollectStaticShapes(shapes);

		b3StackArray<Plane, 32> planes;
		CollectOverlapPlanes(planes, shapes);

		m_isGrounded = false;
		
		for (u32 i = 0; i < planes.Count(); ++i)
		{
			Plane plane = planes[i];

			if (b3Dot(b3Vec3_y, plane.n) > 0.0f)
			{
				m_isGrounded = true;
				break;
			}
		}
	}

	void CollectStaticShapes(b3Array<b3Shape*>& shapes) const
	{
		for (b3Body* b = m_world->GetBodyList().m_head; b; b = b->GetNext())
		{
			if (b == m_characterBody)
			{
				continue;
			}

			if (b->GetType() != e_staticBody)
			{
				continue;
			}

			for (b3Shape* s = b->GetShapeList().m_head; s; s = s->GetNext())
			{
				if (s->GetType() == e_meshShape)
				{
					continue;
				}

				shapes.PushBack(s);
			}
		}
	}

	void CollectOverlapPlanes(b3Array<Plane>& planes,
		const b3Array<b3Shape*>& shapes) const
	{
		b3Sphere sphere2 = GetCharacterSphere();

		for (u32 i = 0; i < shapes.Count(); ++i)
		{
			b3Shape* shape1 = shapes[i];
			b3Body* body1 = shape1->GetBody();
			b3Transform xf1 = body1->GetTransform();

			b3TestSphereOutput output;
			bool overlap = shape1->TestSphere(&output, sphere2, xf1);

			if (overlap == false)
			{
				continue;
			}

			Plane plane;
			plane.n = output.normal;
			plane.p = output.point;

			planes.PushBack(plane);
		}
	}

	void CollectSweepPlanes(b3Array<Plane>& planes,
		const b3Array<b3Shape*>& shapes,
		const b3Vec3& translation2) const
	{
		if (b3LengthSquared(translation2) < B3_EPSILON * B3_EPSILON)
		{
			return;
		}

		b3Transform xf2 = m_characterBody->GetTransform();

		b3ShapeGJKProxy proxy2;
		proxy2.Set(m_characterShape, 0);

		for (u32 i = 0; i < shapes.Count(); ++i)
		{
			b3Shape* shape1 = shapes[i];
			b3Body* body1 = shape1->GetBody();

			b3Transform xf1 = body1->GetTransform();

			b3ShapeGJKProxy proxy1;
			proxy1.Set(shape1, 0);

			b3GJKShapeCastOutput output;
			bool hit = b3GJKShapeCast(&output, xf1, proxy1, xf2, proxy2, translation2);

			if (hit == false)
			{
				continue;
			}

			Plane plane;
			plane.n = output.normal;
			plane.p = output.point;

			planes.PushBack(plane);
		}
	}

	b3Vec3 SolvePositionConstraints(const b3Array<Plane>& planes, const b3Vec3& position) const
	{
		b3Vec3 localCenter = m_characterBody->GetLocalCenter();
		b3Vec3 c = position;
		b3Quat q = m_characterBody->GetOrientation();

		const u32 kMaxPositionIterations = 10;

		for (u32 i = 0; i < kMaxPositionIterations; ++i)
		{
			for (u32 j = 0; j < planes.Count(); ++j)
			{
				Plane nc = planes[j];
				b3Plane plane(nc.n, nc.p);

				b3Transform xf;
				xf.rotation = b3QuatMat33(q);
				xf.position = c - b3Mul(xf.rotation, localCenter);

				b3Sphere sphere;
				sphere.vertex = b3Mul(xf, m_characterShape->m_center);
				sphere.radius = m_characterShape->m_radius;

				float32 separation = b3Distance(sphere.vertex, plane);

				if (separation > sphere.radius)
				{
					continue;
				}

				separation -= sphere.radius;

				float32 C = b3Clamp(B3_BAUMGARTE * (separation + B3_LINEAR_SLOP), -B3_MAX_LINEAR_CORRECTION, 0.0f);
				
				b3Vec3 normal = plane.normal;
				b3Vec3 P = C * normal;

				c -= P;
			}
		}

		return c;
	}

	b3World* m_world;
	b3Body* m_characterBody;
	b3SphereShape* m_characterShape;
	b3Vec3 m_translation;
	bool m_isGrounded;
};

class CharacterTest : public Test
{
public:
	CharacterTest()
	{
		{
			b3BodyDef bdef;
			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.position.Set(-10.0f, 3.0f, 0.0f);

			b3Body* body = m_world.CreateBody(bdef);

			static b3BoxHull boxHull;
			boxHull.Set(2.0f, 2.0f, 5.0f);

			b3HullShape hs;
			hs.m_hull = &boxHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}
		
		{
			b3BodyDef bdef;
			bdef.position.Set(-10.0f, 9.0f, 0.0f);

			b3Body* body = m_world.CreateBody(bdef);

			static b3BoxHull boxHull;
			boxHull.Set(1.0f, 4.0f, 1.0f);

			b3HullShape hs;
			hs.m_hull = &boxHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}
		
		{
			b3BodyDef bdef;
			bdef.position.Set(-10.0f, 2.7f, 7.0f);
			bdef.orientation.Set(b3Vec3_x, 0.25f * B3_PI);

			b3Body* body = m_world.CreateBody(bdef);

			static b3BoxHull boxHull;
			boxHull.Set(2.0f, 0.25f, 3.0f);

			b3HullShape hs;
			hs.m_hull = &boxHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}
		
		{
			b3BodyDef bdef;
			bdef.position.Set(-10.0f, 2.7f, -7.0f);
			bdef.orientation.Set(b3Vec3_x, -0.25f * B3_PI);

			b3Body* body = m_world.CreateBody(bdef);

			static b3BoxHull boxHull;
			boxHull.Set(2.0f, 0.25f, 3.0f);

			b3HullShape hs;
			hs.m_hull = &boxHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.position.Set(10.0f, 3.0f, 0.0f);

			b3Body* body = m_world.CreateBody(bdef);

			static b3BoxHull boxHull;
			boxHull.Set(2.0f, 2.0f, 5.0f);

			b3HullShape hs;
			hs.m_hull = &boxHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}
		
		{
			b3BodyDef bdef;
			bdef.position.Set(10.0f, 9.0f, 0.0f);

			b3Body* body = m_world.CreateBody(bdef);

			static b3BoxHull boxHull;
			boxHull.Set(1.0f, 4.0f, 1.0f);

			b3HullShape hs;
			hs.m_hull = &boxHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.position.Set(10.0f, 2.7f, 7.0f);
			bdef.orientation.Set(b3Vec3_x, 0.25f * B3_PI);

			b3Body* body = m_world.CreateBody(bdef);

			static b3BoxHull boxHull;
			boxHull.Set(2.0f, 0.25f, 3.0f);

			b3HullShape hs;
			hs.m_hull = &boxHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.position.Set(10.0f, 2.7f, -7.0f);
			bdef.orientation.Set(b3Vec3_x, -0.25f * B3_PI);

			b3Body* body = m_world.CreateBody(bdef);

			static b3BoxHull boxHull;
			boxHull.Set(2.0f, 0.25f, 3.0f);

			b3HullShape hs;
			hs.m_hull = &boxHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}
		
		{
			b3BodyDef bdef;
			bdef.position.Set(0.0f, 4.0f, 0.0f);

			b3Body* body = m_world.CreateBody(bdef);

			static b3BoxHull boxHull;
			boxHull.Set(8.0f, 1.0f, 2.0f);

			b3HullShape hs;
			hs.m_hull = &boxHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}
		
		{
			b3BodyDef bdef;
			bdef.type = b3BodyType::e_kinematicBody;
			bdef.position.Set(0.0f, 1.0f, 20.0f);

			b3Body* body = m_world.CreateBody(bdef);

			b3SphereShape sphere;
			sphere.m_center.SetZero();
			sphere.m_radius = 1.0f;

			b3ShapeDef sdef;
			sdef.shape = &sphere;

			b3SphereShape* shape = (b3SphereShape*)body->CreateShape(sdef);

			m_characterController = new CharacterController(shape);
		}
	}

	~CharacterTest()
	{
		delete m_characterController;
	}

	void Step()
	{
		g_draw->DrawString(b3Color_white, "Arrows - Walk");
		g_draw->DrawString(b3Color_white, "Space - Jump");

		float32 dt = g_testSettings->inv_hertz;

		const float32 walkSpeed = 10.0f;
		const float32 jumpSpeed = 300.0f;
		const float32 gravity = 50.0f * 9.8f;
		b3Vec3 velocity = b3Vec3_zero;

		bool isGounded = m_characterController->IsGrounded();

		if (isGounded)
		{
			extern GLFWwindow* g_window;

			bool leftDown = glfwGetKey(g_window, GLFW_KEY_LEFT);
			bool rightDown = glfwGetKey(g_window, GLFW_KEY_RIGHT);
			bool downDown = glfwGetKey(g_window, GLFW_KEY_DOWN);
			bool upDown = glfwGetKey(g_window, GLFW_KEY_UP);
			bool spaceDown = glfwGetKey(g_window, GLFW_KEY_SPACE);

			// Walk
			if (leftDown)
			{
				velocity.x -= walkSpeed;
			}

			if (rightDown)
			{
				velocity.x += walkSpeed;
			}

			if (upDown)
			{
				velocity.z -= walkSpeed;
			}

			if (downDown)
			{
				velocity.z += walkSpeed;
			}

			// Jump
			if (spaceDown)
			{
				velocity.y += jumpSpeed;
			}
		}

		// Integrate gravity
		velocity.y -= dt * gravity;

		// Compute translation
		b3Vec3 translation = dt * velocity;
		
		// Move character
		m_characterController->Move(translation);

		// Step controllers
		m_characterController->Step();

		// Step world
		Test::Step();
	}

	static Test* Create()
	{
		return new CharacterTest();
	}

	CharacterController* m_characterController;
};

#endif