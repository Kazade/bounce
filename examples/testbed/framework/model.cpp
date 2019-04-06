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

#include <testbed/framework/model.h>
#include <testbed/framework/view_model.h>
#include <testbed/framework/test.h>

Model::Model()
{
	m_viewModel = nullptr;
	g_draw = &m_draw;
	g_camera = &m_camera;
	g_profiler = &m_profiler;
	g_profilerSt = &m_profilerSt;
	
#if (PROFILE_JSON == 1)
	g_profilerListener = &m_jsonListener;
#endif

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
	g_profilerSt = nullptr;

#if (PROFILE_JSON == 1)
	g_profilerListener = nullptr;
#endif

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
	b3Ray3 pw = m_camera.ConvertScreenToWorld(ps);

	m_test->MouseLeftDown(pw);
}

void Model::Command_Release_Mouse_Left(const b3Vec2& ps)
{
	b3Ray3 pw = m_camera.ConvertScreenToWorld(ps);

	m_test->MouseLeftUp(pw);
}

void Model::Command_Move_Cursor(const b3Vec2& ps)
{
	b3Ray3 pw = m_camera.ConvertScreenToWorld(ps);

	m_test->MouseMove(pw);
}

void Model::Update()
{
	m_draw.EnableDrawPoints(g_settings->drawPoints);
	m_draw.EnableDrawLines(g_settings->drawLines);
	m_draw.EnableDrawTriangles(g_settings->drawTriangles);
	m_draw.SetViewMatrix(g_camera->BuildViewMatrix());
	m_draw.SetProjectionMatrix(g_camera->BuildProjectionMatrix());

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
		const i32 h = 21;
		const i32 w = 21;

		b3Vec3 vs[h * w];

		b3Vec3 t;
		t.x = -0.5f * float32(w) + 0.5f;
		t.y = 0.0f;
		t.z = -0.5f * float32(h) + 0.5f;

		for (u32 i = 0; i < h; ++i)
		{
			for (u32 j = 0; j < w; ++j)
			{
				u32 iv = i * w + j;

				b3Vec3 v;
				v.x = float32(j);
				v.y = 0.0f;
				v.z = float32(i);

				v += t;
				
				vs[iv] = v;
			}
		}

		b3Color color(0.2f, 0.2f, 0.2f, 1.0f);

		// Left-Right Lines
		u32 hv1 = (h - 1) / 2 * w + 0;
		u32 hv2 = (h - 1) / 2 * w + (w - 1);
		{
			b3Vec3 v1 = vs[hv1];
			b3Vec3 v2 = vs[hv2];

			b3Draw_draw->DrawSegment(v1, v2, b3Color_black);
		}
		
		for (u32 i = 0; i < h; ++i)
		{
			if (i == hv1)
			{
				continue;
			}

			u32 iv1 = i * w + 0;
			u32 iv2 = i * w + (w - 1);

			b3Vec3 v1 = vs[iv1];
			b3Vec3 v2 = vs[iv2];

			b3Draw_draw->DrawSegment(v1, v2, color);
		}

		// Up-Bottom Lines
		u32 wv1 = 0 * w + (w - 1) / 2;
		u32 wv2 = (h - 1) * w + (w - 1) / 2;
		{
			b3Vec3 v1 = vs[wv1];
			b3Vec3 v2 = vs[wv2];

			b3Draw_draw->DrawSegment(v1, v2, b3Color_black);
		}

		for (u32 j = 0; j < w; ++j)
		{
			if (j == wv1)
			{
				continue;
			}

			u32 iv1 = 0 * w + j;
			u32 iv2 = (h - 1) * w + j;

			b3Vec3 v1 = vs[iv1];
			b3Vec3 v2 = vs[iv2];

			b3Draw_draw->DrawSegment(v1, v2, color);
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