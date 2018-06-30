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

#include <testbed/framework/test.h>
#include <testbed/framework/profiler.h>
#include <imgui/imgui.h>

extern u32 b3_allocCalls, b3_maxAllocCalls;
extern u32 b3_convexCalls, b3_convexCacheHits;
extern u32 b3_gjkCalls, b3_gjkIters, b3_gjkMaxIters;
extern bool b3_convexCache;

void b3BeginProfileScope(const char* name)
{
	g_profiler->PushEvent(name);
}

void b3EndProfileScope()
{
	g_profiler->PopEvent();
}

Test::Test() : 
	m_bodyDragger(&m_ray, &m_world),
	m_clothDragger(&m_ray, &m_world)
{
	b3Draw_draw = g_draw;
	b3_convexCache = g_testSettings->convexCache;

	m_world.SetContactListener(this);

	m_ray.origin.SetZero();
	m_ray.direction.Set(0.0f, 0.0f, -1.0f);
	m_ray.fraction = g_camera->m_zFar;

	m_groundHull.Set(50.0f, 1.0f, 50.0f);
	m_groundMesh.BuildTree();
}

Test::~Test()
{
	b3Draw_draw = nullptr;
}

void Test::Step()
{
	b3_convexCache = g_testSettings->convexCache;

	// Step
	float32 dt = g_testSettings->inv_hertz;

	m_world.SetSleeping(g_testSettings->sleep);
	m_world.SetWarmStart(g_testSettings->warmStart);
	m_world.Step(dt, g_testSettings->velocityIterations, g_testSettings->positionIterations);

	// Draw
	u32 drawFlags = 0;
	drawFlags += g_testSettings->drawBounds * b3Draw::e_aabbsFlag;
	drawFlags += g_testSettings->drawShapes * b3Draw::e_shapesFlag;
	drawFlags += g_testSettings->drawCenterOfMasses * b3Draw::e_centerOfMassesFlag;
	drawFlags += g_testSettings->drawJoints * b3Draw::e_jointsFlag;
	drawFlags += g_testSettings->drawContactPoints * b3Draw::e_contactPointsFlag;
	drawFlags += g_testSettings->drawContactNormals * b3Draw::e_contactNormalsFlag;
	drawFlags += g_testSettings->drawContactTangents * b3Draw::e_contactTangentsFlag;
	drawFlags += g_testSettings->drawContactPolygons * b3Draw::e_contactPolygonsFlag;

	g_draw->SetFlags(drawFlags);
	
	m_world.Draw();

	g_draw->Flush();

	if (g_settings->drawTriangles)
	{
		g_draw->DrawSolidShapes(m_world);
	}

	if (g_settings->drawStats)
	{
		g_draw->DrawString(b3Color_white, "Bodies %d", m_world.GetBodyList().m_count);
		g_draw->DrawString(b3Color_white, "Joints %d", m_world.GetJointList().m_count);
		g_draw->DrawString(b3Color_white, "Contacts %d", m_world.GetContactList().m_count);

		float32 avgGjkIters = 0.0f;
		if (b3_gjkCalls > 0)
		{
			avgGjkIters = float32(b3_gjkIters) / float32(b3_gjkCalls);
		}

		g_draw->DrawString(b3Color_white, "GJK Calls %d", b3_gjkCalls);
		g_draw->DrawString(b3Color_white, "GJK Iterations %d (%d) (%f)", b3_gjkIters, b3_gjkMaxIters, avgGjkIters);

		float32 convexCacheHitRatio = 0.0f;
		if (b3_convexCalls > 0)
		{
			convexCacheHitRatio = float32(b3_convexCacheHits) / float32(b3_convexCalls);
		}

		g_draw->DrawString(b3Color_white, "Convex Calls %d", b3_convexCalls);
		g_draw->DrawString(b3Color_white, "Convex Cache Hits %d (%f)", b3_convexCacheHits, convexCacheHitRatio);
		g_draw->DrawString(b3Color_white, "Frame Allocations %d (%d)", b3_allocCalls, b3_maxAllocCalls);
	}
}

void Test::MouseMove(const b3Ray3& pw)
{
	m_ray = pw;

	if (m_bodyDragger.IsDragging() == true)
	{
		m_bodyDragger.Drag();
	}
	
	if (m_clothDragger.IsDragging() == true)
	{
		m_clothDragger.Drag();
	}
}

void Test::MouseLeftDown(const b3Ray3& pw)
{
	if (m_bodyDragger.IsDragging() == false)
	{
		if (m_bodyDragger.StartDragging() == true)
		{
			BeginDragging();
		}
	}

	if (m_clothDragger.IsDragging() == false)
	{
		if (m_clothDragger.StartDragging() == true)
		{
			BeginDragging();
		}
	}
}

void Test::MouseLeftUp(const b3Ray3& pw)
{
	if (m_bodyDragger.IsDragging() == true)
	{
		m_bodyDragger.StopDragging();

		EndDragging();
	}
	
	if (m_clothDragger.IsDragging() == true)
	{
		m_clothDragger.StopDragging();

		EndDragging();
	}
}