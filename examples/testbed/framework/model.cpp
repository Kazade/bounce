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
#include <testbed/framework/view_model.h>
#include <testbed/framework/test.h>

Model::Model()
{
	m_viewModel = nullptr;
	g_draw = &m_draw;
	g_camera = &m_camera;
	g_profiler = &m_profiler;
	g_profilerRecorder = &m_profilerListener.m_recorderProfiler;
	g_profilerListener = &m_profilerListener;

	m_test = nullptr;

	glFrontFace(GL_CCW);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
	glClearDepth(1.0f);

	Action_ResetCamera();

	m_setTest = true;
	m_pause = true;
	m_singlePlay = false;
}

Model::~Model()
{
	g_draw = nullptr;
	g_camera = nullptr;
	g_profiler = nullptr;
	g_profilerRecorder = nullptr;
	g_profilerListener = nullptr;

	delete m_test;
}

void Model::Action_SaveTest()
{
	m_test->Save();
}

void Model::Command_Press_Key(int button)
{
	m_test->KeyDown(button);
}

void Model::Command_Release_Key(int button)
{
	m_test->KeyUp(button);
}

void Model::Command_Press_Mouse_Left(const b3Vec2& ps)
{
	Ray3 pw = m_camera.ConvertScreenToWorld(ps);

	m_test->MouseLeftDown(pw);
}

void Model::Command_Release_Mouse_Left(const b3Vec2& ps)
{
	Ray3 pw = m_camera.ConvertScreenToWorld(ps);

	m_test->MouseLeftUp(pw);
}

void Model::Command_Move_Cursor(const b3Vec2& ps)
{
	Ray3 pw = m_camera.ConvertScreenToWorld(ps);

	m_test->MouseMove(pw);
}

void Model::Update()
{
	g_drawFlags = 0;
	g_drawFlags += g_settings->drawPoints * DrawFlags::e_pointsFlag;
	g_drawFlags += g_settings->drawLines * DrawFlags::e_linesFlag;
	g_drawFlags += g_settings->drawTriangles * DrawFlags::e_trianglesFlag;

	glViewport(0, 0, GLsizei(m_camera.m_width), GLsizei(m_camera.m_height));
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (m_setTest)
	{
		delete m_test;
		m_test = g_tests[g_settings->testID].create();
		m_setTest = false;
		m_pause = true;
		Action_ResetCamera();
	}
	
	if (g_settings->drawGrid)
	{
		b3Color color(0.2f, 0.2f, 0.2f, 1.0f);

		b3Vec3 pn(0.0f, 1.0f, 0.0f);
		b3Vec3 p(0.0f, 0.0f, 0.0f);
		m_draw.DrawCircle(pn, p, 1.0f, color);

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

				m_draw.DrawPolygon(vs, 4, color);
			}
		}
	}

	//
	if (m_pause)
	{
		if (m_singlePlay)
		{
			// !
			g_testSettings->inv_hertz = g_testSettings->hertz > 0.0f ? 1.0f / g_testSettings->hertz : 0.0f;
			m_singlePlay = false;
		}
		else
		{
			// !
			g_testSettings->inv_hertz = 0.0f;
		}
	}
	else
	{
		// !
		g_testSettings->inv_hertz = g_testSettings->hertz > 0.0f ? 1.0f / g_testSettings->hertz : 0.0f;
	}

	m_test->Step();

	m_draw.Flush();
}