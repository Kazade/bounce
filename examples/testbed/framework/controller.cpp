#include <testbed/framework/controller.h>
#include <testbed/framework/model.h>
#include <testbed/framework/view.h>

// !
#include <glfw/glfw3.h>

// !
static inline b3Vec2 GetCursorPosition()
{
	extern GLFWwindow* g_window;

	double x, y;
	glfwGetCursorPos(g_window, &x, &y);

	return b3Vec2(float32(x), float32(y));
}

Controller::Controller(Model* model, View* view)
{
	m_model = model;
	m_view = view;
	
	m_leftDown = false;
	m_rightDown = false;
	m_shiftDown = false;
	m_ps0.SetZero();
}

Controller::~Controller()
{

}

void Controller::Event_SetWindowSize(u32 w, u32 h)
{
	m_model->Command_ResizeCamera(float32(w), float32(h));
}

void Controller::Event_Press_Key(int button)
{
	if (button == GLFW_KEY_LEFT_SHIFT)
	{
		m_shiftDown = true;
	}

	if (m_shiftDown)
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

void Controller::Event_Release_Key(int button)
{
	if (button == GLFW_KEY_LEFT_SHIFT)
	{
		m_shiftDown = false;
	}

	if (m_shiftDown)
	{

	}
	else
	{
		m_model->Command_Release_Key(button);
	}
}

void Controller::Event_Press_Mouse(int button)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		m_leftDown = true;

		if (!m_shiftDown)
		{
			m_model->Command_Press_Mouse_Left(GetCursorPosition());
		}
	}

	if (button == GLFW_MOUSE_BUTTON_RIGHT)
	{
		m_rightDown = true;
	}
}

void Controller::Event_Release_Mouse(int button)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		m_leftDown = false;

		if (!m_shiftDown)
		{
			m_model->Command_Release_Mouse_Left(GetCursorPosition());
		}
	}

	if (button == GLFW_MOUSE_BUTTON_RIGHT)
	{
		m_rightDown = false;
	}
}

void Controller::Event_Move_Cursor(float32 x, float32 y)
{
	b3Vec2 ps;
	ps.Set(float32(x), float32(y));

	b3Vec2 dp = ps - m_ps0;
	m_ps0 = ps;

	float32 ndx = b3Clamp(dp.x, -1.0f, 1.0f);
	float32 ndy = b3Clamp(dp.y, -1.0f, 1.0f);

	if (m_shiftDown)
	{
		if (m_leftDown)
		{
			float32 ax = -0.005f * B3_PI * ndx;
			float32 ay = -0.005f * B3_PI * ndy;
			
			m_model->Command_RotateCameraY(ax);
			m_model->Command_RotateCameraX(ay);
		}

		if (m_rightDown)
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

void Controller::Event_Scroll(float32 dx, float32 dy)
{
	if (m_shiftDown)
	{
		float32 ny = b3Clamp(dy, -1.0f, 1.0f);
		m_model->Command_ZoomCamera(1.0f * ny);
	}
}