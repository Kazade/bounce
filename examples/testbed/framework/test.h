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

#ifndef TEST_H
#define TEST_H

#include <glfw/glfw3.h>
#include <bounce/bounce.h>

#include <testbed/framework/body_dragger.h>
#include <testbed/framework/cloth_dragger.h>
#include <testbed/framework/softbody_dragger.h>

#include <testbed/framework/draw.h>
#include <testbed/framework/view_model.h>

inline float RandomFloat(scalar a, scalar b)
{
	float x = float(rand()) / float(RAND_MAX);
	float diff = b - a;
	float r = x * diff;
	return a + r;
}

class Test : public b3ContactListener
{
public:
	Test();
	virtual ~Test();

	virtual void Save() { }

	virtual void Step();

	virtual void MouseMove(const b3Ray3& pw);
	virtual void MouseLeftDown(const b3Ray3& pw);
	virtual void MouseLeftUp(const b3Ray3& pw);
	virtual void KeyDown(int button) { }
	virtual void KeyUp(int button) { }

	virtual void BeginDragging() { }
	virtual void EndDragging() { }

	void BeginContact(b3Contact* c) override { }
	void EndContact(b3Contact* c) override { }
	void PreSolve(b3Contact* c) override { }

	b3Ray3 m_ray;
	
	b3World m_world;
	b3BodyDragger m_bodyDragger;

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