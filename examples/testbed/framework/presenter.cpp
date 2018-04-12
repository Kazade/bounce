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

#include <testbed/framework/presenter.h>
#include <testbed/framework/model.h>
#include <testbed/framework/view.h>

// !
static inline b3Vec2 GetCursorPosition()
{
	extern GLFWwindow* g_window;

	double x, y;
	glfwGetCursorPos(g_window, &x, &y);

	return b3Vec2(float32(x), float32(y));
}

Presenter::Presenter(Model* model, View* view)
{
	m_model = model;
	m_view = view;
}

Presenter::~Presenter()
{

}

void Presenter::Event_SetWindowSize(float w, float h)
{
	m_model->Command_ResizeCamera(w, h);
}

void Presenter::Event_Press_Key(int button)
{
	if (m_view->m_shiftDown)
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

void Presenter::Event_Release_Key(int button)
{
	if (!m_view->m_shiftDown)
	{
		m_model->Command_Release_Key(button);
	}
}

void Presenter::Event_Press_Mouse(int button)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		if (!m_view->m_shiftDown)
		{
			m_model->Command_Press_Mouse_Left(GetCursorPosition());
		}
	}
}

void Presenter::Event_Release_Mouse(int button)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		if (!m_view->m_shiftDown)
		{
			m_model->Command_Release_Mouse_Left(GetCursorPosition());
		}
	}
}

void Presenter::Event_Move_Cursor(float x, float y)
{
	b3Vec2 ps;
	ps.Set(x, y);

	b3Vec2 dp = ps - m_view->m_ps0;

	float32 ndx = b3Clamp(dp.x, -1.0f, 1.0f);
	float32 ndy = b3Clamp(dp.y, -1.0f, 1.0f);

	if (m_view->m_shiftDown)
	{
		if (m_view->m_leftDown)
		{
			float32 ax = -0.005f * B3_PI * ndx;
			float32 ay = -0.005f * B3_PI * ndy;

			m_model->Command_RotateCameraY(ax);
			m_model->Command_RotateCameraX(ay);
		}

		if (m_view->m_rightDown)
		{
			float32 tx = 0.2f * ndx;
			float32 ty = -0.2f * ndy;

			m_model->Command_TranslateCameraX(tx);
			m_model->Command_TranslateCameraY(ty);
		}
	}
	else
	{
		m_model->Command_Move_Cursor(GetCursorPosition());
	}
}

void Presenter::Event_Scroll(float dx, float dy)
{
	if (m_view->m_shiftDown)
	{
		float32 ny = b3Clamp(dy, -1.0f, 1.0f);
		m_model->Command_ZoomCamera(1.0f * ny);
	}
}
