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

#include <testbed/tests/test.h>

extern u32 b3_allocCalls, b3_maxAllocCalls;
extern u32 b3_gjkCalls, b3_gjkIters, b3_gjkMaxIters;
extern u32 b3_convexCalls, b3_convexCacheHits;
extern bool b3_enableConvexCache;
extern b3Draw* b3_debugDraw;

extern Settings g_settings;
extern DebugDraw* g_debugDraw;
extern Camera g_camera;
extern Profiler* g_profiler;

Test::Test()
{
	b3_allocCalls = 0;
	b3_gjkCalls = 0;
	b3_gjkIters = 0;
	b3_gjkMaxIters = 0;
	b3_convexCalls = 0;
	b3_convexCacheHits = 0;
	b3_enableConvexCache = g_settings.convexCache;
	b3_debugDraw = g_debugDraw;

	m_world.SetContactListener(this);

	g_camera.m_q = b3Quat(b3Vec3(0.0f, 1.0f, 0.0f), 0.15f * B3_PI);
	g_camera.m_q = g_camera.m_q * b3Quat(b3Vec3(1.0f, 0.0f, 0.0f), -0.15f * B3_PI);
	g_camera.m_zoom = 50.0f;
	g_camera.m_center.SetZero();
	
	m_rayHit.shape = NULL;
	m_mouseJoint = NULL;

	{
		b3Transform xf;
		xf.position.SetZero();
		xf.rotation = b3Diagonal(50.0f, 1.0f, 50.0f);
		m_groundHull.SetTransform(xf);
	}

	{
		b3Transform xf;
		xf.position.SetZero();
		xf.rotation = b3Diagonal(1.0f, 1.0f, 1.0f);
		m_boxHull.SetTransform(xf);
	}

	{
		b3Transform xf;
		xf.position.SetZero();
		xf.rotation = b3Diagonal(1.0f, 5.0f, 1.0f);
		m_tallHull.SetTransform(xf);
	}

	{
		b3Transform xf;
		xf.position.SetZero();
		xf.rotation = b3Diagonal(2.0f, 4.0f, 0.5f);
		m_doorHull.SetTransform(xf);
	}
	
	{
		b3Transform xf;
		xf.position.SetZero();
		xf.rotation = b3Diagonal(25.0f, 0.5f, 25.0f);
		m_rampHull.SetTransform(xf);
	}

	{
		b3Transform xf;
		xf.position.SetZero();
		xf.rotation = b3Diagonal(1.0f, 0.5f, 3.0f);
		m_plankHull.SetTransform(xf);
	}
	
	{
		b3Transform xf;
		xf.position.SetZero();
		xf.rotation = b3Diagonal(4.05f, 2.0f * B3_LINEAR_SLOP, 4.05f);
		m_thinHull.SetTransform(xf);
	}

	{
		const u32 w = 5;
		const u32 h = 5;

		b3Vec3 t;
		t.x = -0.5f * float32(w);
		t.y = 0.0f;
		t.z = -0.5f * float32(h);

		b3Mesh* mesh = m_meshes + e_clothMesh;

		mesh->vertexCount = w * h;
		mesh->vertices = (b3Vec3*)b3Alloc(mesh->vertexCount * sizeof(b3Vec3));

		for (u32 i = 0; i < w; ++i)
		{
			for (u32 j = 0; j < h; ++j)
			{
				u32 v1 = i * w + j;

				b3Vec3 v;
				v.x = float32(i);
				v.y = RandomFloat(0.0f, 0.5f);
				v.z = float32(j);

				v += t;

				mesh->vertices[v1] = v;
			}
		}

		mesh->triangleCount = 2 * (w - 1) * (h - 1);
		mesh->triangles = (b3Triangle*)b3Alloc(mesh->triangleCount * sizeof(b3Triangle));

		u32 triangleCount = 0;
		for (u32 i = 0; i < w - 1; ++i)
		{
			for (u32 j = 0; j < h - 1; ++j)
			{
				u32 v1 = i * w + j;
				u32 v2 = (i + 1) * w + j;
				u32 v3 = (i + 1) * w + (j + 1);
				u32 v4 = i * w + (j + 1);

				B3_ASSERT(triangleCount < mesh->triangleCount);
				b3Triangle* t1 = mesh->triangles + triangleCount;
				++triangleCount;

				t1->v1 = v3;
				t1->v2 = v2;
				t1->v3 = v1;

				B3_ASSERT(triangleCount < mesh->triangleCount);
				b3Triangle* t2 = mesh->triangles + triangleCount;
				++triangleCount;

				t2->v1 = v1;
				t2->v2 = v4;
				t2->v3 = v3;
			}
		}

		B3_ASSERT(triangleCount == mesh->triangleCount);

		mesh->BuildTree();
	}

	{
		const u32 w = 50;
		const u32 h = 50;

		b3Vec3 t;
		t.x = -0.5f * float32(w);
		t.y = 0.0f;
		t.z = -0.5f * float32(h);

		b3Mesh* mesh = m_meshes + e_gridMesh;

		mesh->vertexCount = w * h;
		mesh->vertices = (b3Vec3*)b3Alloc(mesh->vertexCount * sizeof(b3Vec3));

		for (u32 i = 0; i < w; ++i)
		{
			for (u32 j = 0; j < h; ++j)
			{
				u32 v1 = i * w + j;

				b3Vec3 v;
				v.x = float32(i);
				v.y = 0.0f;
				v.z = float32(j);

				v += t;

				mesh->vertices[v1] = v;
			}
		}

		// 2 triangles per quad
		mesh->triangleCount = 2 * (w - 1) * (h - 1);
		mesh->triangles = (b3Triangle*)b3Alloc(mesh->triangleCount * sizeof(b3Triangle));

		u32 triangleCount = 0;
		for (u32 i = 0; i < w - 1; ++i)
		{
			for (u32 j = 0; j < h - 1; ++j)
			{
				u32 v1 = i * w + j;
				u32 v2 = (i + 1) * w + j;
				u32 v3 = (i + 1) * w + (j + 1);
				u32 v4 = i * w + (j + 1);

				B3_ASSERT(triangleCount < mesh->triangleCount);
				b3Triangle* t1 = mesh->triangles + triangleCount;
				++triangleCount;

				t1->v1 = v3;
				t1->v2 = v2;
				t1->v3 = v1;

				B3_ASSERT(triangleCount < mesh->triangleCount);
				b3Triangle* t2 = mesh->triangles + triangleCount;
				++triangleCount;

				t2->v1 = v1;
				t2->v2 = v4;
				t2->v3 = v3;
			}
		}

		B3_ASSERT(triangleCount == mesh->triangleCount);

		mesh->BuildTree();
	}

	{
		const u32 w = 50;
		const u32 h = 50;

		b3Vec3 t;
		t.x = -0.5f * float32(w);
		t.y = 0.0f;
		t.z = -0.5f * float32(h);

		b3Mesh* mesh = m_meshes + e_terrainMesh;
		
		mesh->vertexCount = w * h;
		mesh->vertices = (b3Vec3*)b3Alloc(mesh->vertexCount * sizeof(b3Vec3));
		
		for (u32 i = 0; i < w; ++i)
		{
			for (u32 j = 0; j < h; ++j)
			{
				u32 v1 = i * w + j;
				
				b3Vec3 v;
				v.x = 2.0f * float32(i);
				v.y = RandomFloat(0.0f, 0.5f);
				v.z = 2.0f *float32(j);

				v += t;

				mesh->vertices[v1] = v;
			}
		}

		mesh->triangleCount = 2 * (w - 1) * (h - 1);
		mesh->triangles = (b3Triangle*)b3Alloc(mesh->triangleCount * sizeof(b3Triangle));

		u32 triangleCount = 0;
		for (u32 i = 0; i < w - 1; ++i)
		{
			for (u32 j = 0; j < h - 1; ++j)
			{
				u32 v1 = i * w + j;
				u32 v2 = (i + 1) * w + j;
				u32 v3 = (i + 1) * w + (j + 1);
				u32 v4 = i * w + (j + 1);

				B3_ASSERT(triangleCount < mesh->triangleCount);
				b3Triangle* t1 = mesh->triangles + triangleCount;
				++triangleCount;

				t1->v1 = v3;
				t1->v2 = v2;
				t1->v3 = v1;

				B3_ASSERT(triangleCount < mesh->triangleCount);
				b3Triangle* t2 = mesh->triangles + triangleCount;
				++triangleCount;

				t2->v1 = v1;
				t2->v2 = v4;
				t2->v3 = v3;
			}
		}

		B3_ASSERT(triangleCount == mesh->triangleCount);

		mesh->BuildTree();
	}
}

Test::~Test()
{
	for (u32 i = 0; i < e_maxMeshes; ++i)
	{
		b3Free(m_meshes[i].vertices);
		b3Free(m_meshes[i].triangles);
	}
}

void Test::BeginContact(b3Contact* contact)
{
}

void Test::EndContact(b3Contact* contact)
{

}

void Test::PreSolve(b3Contact* contact)
{

}

void Test::Step()
{
	float32 dt = g_settings.hertz > 0.0f ? 1.0f / g_settings.hertz : 0.0f;

	if (g_settings.pause)
	{
		if (g_settings.singleStep)
		{
			g_settings.singleStep = false;
		}
		else
		{
			dt = 0.0f;
		}
	}

	b3_allocCalls = 0;
	b3_gjkCalls = 0;
	b3_gjkIters = 0;
	b3_gjkMaxIters = 0;
	b3_convexCalls = 0;
	b3_convexCacheHits = 0;
	b3_enableConvexCache = g_settings.convexCache;
	
	// Step
	ProfileBegin();

	m_world.SetSleeping(g_settings.sleep);
	m_world.SetWarmStart(g_settings.warmStart);
	m_world.Step(dt, g_settings.velocityIterations, g_settings.positionIterations);

	ProfileEnd();
	
	g_debugDraw->Submit();

	// Draw World
	u32 drawFlags = 0;
	drawFlags += g_settings.drawBounds * b3Draw::e_aabbsFlag;
	drawFlags += g_settings.drawVerticesEdges * b3Draw::e_shapesFlag;
	drawFlags += g_settings.drawCenterOfMasses * b3Draw::e_centerOfMassesFlag;
	drawFlags += g_settings.drawJoints * b3Draw::e_jointsFlag;
	drawFlags += g_settings.drawContactPoints * b3Draw::e_contactPointsFlag;
	drawFlags += g_settings.drawContactNormals * b3Draw::e_contactNormalsFlag;
	drawFlags += g_settings.drawContactTangents * b3Draw::e_contactTangentsFlag;

	g_debugDraw->SetFlags(drawFlags);
	m_world.DebugDraw();
	
	if (m_mouseJoint)
	{
		b3Shape* shape = m_rayHit.shape;
		b3Body* body = shape->GetBody();

		b3Vec3 n = body->GetWorldVector(m_rayHit.normal);
		b3Vec3 p = body->GetWorldPoint(m_rayHit.point);
		
		g_debugDraw->DrawSolidCircle(n, p + 0.05f * n, 1.0f, b3Color_white);
	}

	g_debugDraw->Submit();
	
	if (g_settings.drawFaces)
	{
		g_debugDraw->Draw(m_world);
	}

	// Draw Statistics
	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
	ImGui::Begin("Log", NULL, ImVec2(0, 0), 0.0f, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);

	if (g_settings.pause)
	{
		ImGui::Text("*PAUSED*");
	}

	if (g_settings.drawStats)
	{
		ImGui::Text("Bodies %d", m_world.GetBodyList().m_count);
		ImGui::Text("Joints %d", m_world.GetJointList().m_count);
		ImGui::Text("Contacts %d", m_world.GetContactList().m_count);

		float32 avgGjkIters = 0.0f;
		if (b3_gjkCalls > 0)
		{
			avgGjkIters = float32(b3_gjkIters) / float32(b3_gjkCalls);
		}

		ImGui::Text("GJK Calls %d", b3_gjkCalls);
		ImGui::Text("GJK Iterations %d (%d) (%f)", b3_gjkIters, b3_gjkMaxIters, avgGjkIters);

		float32 convexCacheHitRatio = 0.0f;
		if (b3_convexCalls > 0)
		{
			convexCacheHitRatio = float32(b3_convexCacheHits) / float32(b3_convexCalls);
		}

		ImGui::Text("Convex Calls %d", b3_convexCalls);
		ImGui::Text("Convex Cache Hits %d (%f)", b3_convexCacheHits, convexCacheHitRatio);
		ImGui::Text("Frame Allocations %d (%d)", b3_allocCalls, b3_maxAllocCalls);
	}

	if (g_settings.drawProfile)
	{
		for (u32 i = 0; i < g_profiler->m_records.Count(); ++i)
		{
			const ProfileRecord& r = g_profiler->m_records[i];
			ImGui::Text("%s %.4f (%.4f) [ms]", r.name, r.elapsed, r.maxElapsed);
		}
	}
	
	g_profiler->Clear();

	ImGui::End();
}

void Test::MouseMove(const Ray3& pw)
{
	if (m_mouseJoint)
	{
		float32 t = m_rayHit.fraction;
		float32 w1 = 1.0f - t;
		float32 w2 = t;

		b3Vec3 target = w1 * pw.Start() + w2 * pw.End();
		m_mouseJoint->SetTarget(target);
	}
}

void Test::MouseLeftDown(const Ray3& pw)
{
	// Clear the current hit
	m_rayHit.shape = NULL;
	if (m_mouseJoint)
	{
		b3Body* groundBody = m_mouseJoint->GetBodyA();
		
		m_world.DestroyJoint(m_mouseJoint);
		m_mouseJoint = NULL;
		
		m_world.DestroyBody(groundBody);
	}

	// Perform the ray cast
	b3Vec3 p1 = pw.Start();
	b3Vec3 p2 = pw.End();

	b3RayCastSingleOutput out;
	if (m_world.RayCastSingle(&out, p1, p2))
	{
		b3Shape* shape = out.shape;
		b3Body* body = shape->GetBody();
		
		m_rayHit.shape = out.shape;
		m_rayHit.point = body->GetLocalPoint(out.point);
		m_rayHit.normal = body->GetLocalVector(out.normal);
		m_rayHit.fraction = out.fraction;

		RayHit();
	}
}

void Test::MouseLeftUp(const Ray3& pw)
{
	m_rayHit.shape = NULL;
	if (m_mouseJoint)
	{
		b3Body* groundBody = m_mouseJoint->GetBodyA();		
		
		m_world.DestroyJoint(m_mouseJoint);
		m_mouseJoint = NULL;
		
		m_world.DestroyBody(groundBody);
	}
}

void Test::RayHit()
{
	b3BodyDef bdef;
	b3Body* bodyA = m_world.CreateBody(bdef);
	b3Body* bodyB = m_rayHit.shape->GetBody();
	
	b3MouseJointDef def;
	def.bodyA = bodyA;
	def.bodyB = bodyB;
	def.target = bodyB->GetWorldPoint(m_rayHit.point);
	def.maxForce = 2000.0f * bodyB->GetMass();

	m_mouseJoint = (b3MouseJoint*)m_world.CreateJoint(def);
	bodyB->SetAwake(true);
}
