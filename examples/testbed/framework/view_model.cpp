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

#include <testbed/framework/view_model.h>
#include <testbed/framework/model.h>
#include <testbed/framework/view.h>
#include <testbed/framework/test.h>
#include <glfw/glfw3.h>

TestSettings* g_testSettings = nullptr;
Settings* g_settings = nullptr;

ViewModel::ViewModel(Model* model, View* view)
{
	m_model = model;
	assert(m_model->m_viewModel == nullptr);
	m_model->m_viewModel = this;

	m_view = view;
	assert(m_view->m_viewModel == nullptr);
	m_view->m_viewModel = this;

	g_settings = &m_settings;
	g_testSettings = &m_testSettings;
}

ViewModel::~ViewModel()
{
	g_settings = nullptr;
	g_testSettings = nullptr;
}

void ViewModel::Action_SaveTest()
{
	m_model->Action_SaveTest();
}

void ViewModel::Action_SetTest()
{
	m_model->Action_SetTest();
}

void ViewModel::Action_PreviousTest()
{
	m_settings.testID = b3Clamp(m_settings.testID - 1, 0, int(g_testCount) - 1);
	m_model->Action_SetTest();
}

void ViewModel::Action_NextTest()
{
	m_settings.testID = b3Clamp(m_settings.testID + 1, 0, int(g_testCount) - 1);
	m_model->Action_SetTest();
}

void ViewModel::Action_PlayPause()
{
	m_model->Action_PlayPause();
}

void ViewModel::Action_SinglePlay()
{
	m_model->Action_SinglePlay();
}

void ViewModel::Action_ResetCamera()
{
	m_model->Action_ResetCamera();
}

void ViewModel::Event_SetWindowSize(int w, int h)
{
	m_model->Command_ResizeCamera(scalar(w), scalar(h));
}

void ViewModel::Event_Press_Key(int button)
{
	bool shiftDown = glfwGetKey(m_view->m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
	if (shiftDown)
	{
		if (button == GLFW_KEY_DOWN)
		{
			m_model->Command_ZoomCamera(1.0f);
		}

		if (button == GLFW_KEY_UP)
		{
			m_model->Command_ZoomCamera(-1.0f);
		}
	}
	else
	{
		m_model->Command_Press_Key(button);
	}
}

void ViewModel::Event_Release_Key(int button)
{
	bool shiftDown = glfwGetKey(m_view->m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
	if (!shiftDown)
	{
		m_model->Command_Release_Key(button);
	}
}

void ViewModel::Event_Press_Mouse(int button)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		bool shiftDown = glfwGetKey(m_view->m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
		if (!shiftDown)
		{
			m_model->Command_Press_Mouse_Left(m_view->GetCursorPosition());
		}
	}
}

void ViewModel::Event_Release_Mouse(int button)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		bool shiftDown = glfwGetKey(m_view->m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
		if (!shiftDown)
		{
			m_model->Command_Release_Mouse_Left(m_view->GetCursorPosition());
		}
	}
}

void ViewModel::Event_Move_Cursor(float x, float y)
{
	b3Vec2 ps;
	ps.Set(x, y);

	b3Vec2 dp = ps - m_view->m_ps0;

	b3Vec2 n = b3Normalize(dp);

	bool shiftDown = glfwGetKey(m_view->m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
	bool leftDown = glfwGetMouseButton(m_view->m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
	bool rightDown = glfwGetMouseButton(m_view->m_window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

	if (shiftDown)
	{
		if (leftDown)
		{
			scalar ax = -0.005f * B3_PI * n.x;
			scalar ay = -0.005f * B3_PI * n.y;

			m_model->Command_RotateCameraY(ax);
			m_model->Command_RotateCameraX(ay);
		}

		if (rightDown)
		{
			scalar tx = 0.2f * n.x;
			scalar ty = -0.2f * n.y;

			m_model->Command_TranslateCameraX(tx);
			m_model->Command_TranslateCameraY(ty);
		}
	}
	else
	{
		m_model->Command_Move_Cursor(ps);
	}
}

void ViewModel::Event_Scroll(float dx, float dy)
{
	b3Vec2 n(dx, dy);
	n.Normalize();

	bool shiftDown = glfwGetKey(m_view->m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
	if (shiftDown)
	{
		m_model->Command_ZoomCamera(1.0f * n.y);
	}
}
