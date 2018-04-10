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

#include <testbed/framework/model.h>

Model::Model()
{
	g_debugDraw = &m_debugDraw;
	g_camera = &m_camera;
	g_overlayName = "overlay";
	g_profiler = &m_profiler;
	g_profilerListener = &m_profilerListener;
	g_settings = &m_settings;

	m_test = nullptr;

	glFrontFace(GL_CCW);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
	glClearDepth(1.0f);

	m_camera.m_q = b3QuatRotationX(-0.125f * B3_PI);
	m_camera.m_center.SetZero();
	m_camera.m_zoom = 20.0f;
}

Model::~Model()
{
	g_debugDraw = nullptr;
	g_camera = nullptr;
	g_overlayName = nullptr;
	g_profiler = nullptr;
	g_profilerListener = nullptr;
	g_settings = nullptr;

	delete m_test;
}

void Model::Command_Step()
{
	if (m_settings.testID != m_settings.lastTestID)
	{
		delete m_test;
		m_settings.lastTestID = m_settings.testID;
		m_test = g_tests[m_settings.testID].create();
		m_settings.pause = true;
	}
	
	glViewport(0, 0, GLsizei(m_camera.m_width), GLsizei(m_camera.m_height));
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (m_settings.pause)
	{
		m_debugDraw.DrawString(b3Color_white, "*PAUSED*");
	}
	else
	{
		m_debugDraw.DrawString(b3Color_white, "*PLAYING*");
	}

	if (m_settings.drawGrid)
	{
		b3Color color(0.2f, 0.2f, 0.2f, 1.0f);

		b3Vec3 pn(0.0f, 1.0f, 0.0f);
		b3Vec3 p(0.0f, 0.0f, 0.0f);
		m_debugDraw.DrawCircle(pn, p, 1.0f, color);

		int n = 20;

		b3Vec3 t;
		t.x = -0.5f * float32(n);
		t.y = 0.0f;
		t.z = -0.5f * float32(n);

		for (int i = 0; i < n; i += 1)
		{
			for (int j = 0; j < n; j += 1)
			{
				b3Vec3 vs[4];
				vs[0] = b3Vec3((float32)i, 0.0f, (float32)j);
				vs[1] = b3Vec3((float32)i, 0.0f, (float32)j + 1);
				vs[2] = b3Vec3((float32)i + 1, 0.0f, (float32)j + 1);
				vs[3] = b3Vec3((float32)i + 1, 0.0f, (float32)j);

				vs[0] += t;
				vs[1] += t;
				vs[2] += t;
				vs[3] += t;

				m_debugDraw.DrawPolygon(vs, 4, color);
			}
		}
	}

	//
	m_settings.inv_hertz = m_settings.hertz != 0.0f ? 1.0f / m_settings.hertz : 0.0f;

	if (m_settings.pause)
	{
		if (m_settings.singleStep)
		{
			m_settings.singleStep = false;
		}
		else
		{
			m_settings.inv_hertz = 0.0f;
		}
	}

	m_profiler.Begin();

	m_test->Step();

	m_profiler.End(g_profilerListener);

	if (m_settings.drawProfile)
	{
		const b3Array<ProfilerRecord>& records = m_profilerListener.m_recorderProfiler.GetRecords();
		for (u32 i = 0; i < records.Count(); ++i)
		{
			const ProfilerRecord& r = records[i];
			m_debugDraw.DrawString(b3Color_white, "%s %.4f (%.4f) [ms]", r.name, r.elapsed, r.maxElapsed);
		}
	}

	m_debugDraw.Submit();
}