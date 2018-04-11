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

#if defined (U_OPENGL_2)
	#include <glad_2/glad.h>
#elif defined (U_OPENGL_4)
	#include <glad_4/glad.h>
#else
	// error
#endif

#include <testbed/framework/model.h>
#include <testbed/framework/view.h>
#include <testbed/framework/controller.h>

//
GLFWwindow* g_window;

//
Model* g_model;
View* g_view;
Controller* g_controller;

static void WindowSize(GLFWwindow* ww, int w, int h)
{
	g_controller->Event_SetWindowSize(u32(w), u32(h));
}

static void CursorMove(GLFWwindow* w, double x, double y)
{
	g_controller->Event_Move_Cursor(float32(x), float32(y));
}

static void WheelScroll(GLFWwindow* w, double dx, double dy)
{
	g_controller->Event_Scroll(float32(dx), float32(dy));
}

static void MouseButton(GLFWwindow* w, int button, int action, int mods)
{
	switch (action)
	{
	case GLFW_PRESS:
	{
		g_controller->Event_Press_Mouse(button);
		break;
	}
	case GLFW_RELEASE:
	{
		g_controller->Event_Release_Mouse(button);
		break;
	}
	default:
	{
		break;
	}
	}
}

static void KeyButton(GLFWwindow* w, int button, int scancode, int action, int mods)
{
	switch (action)
	{
	case GLFW_PRESS:
	{
		g_controller->Event_Press_Key(button);
		
		break;
	}
	case GLFW_RELEASE:
	{
		g_controller->Event_Release_Key(button);
		break;
	}
	default:
	{
		break;
	}
	}
}

static void Run()
{
	int w, h;
	glfwGetWindowSize(g_window, &w, &h);
	g_controller->Event_SetWindowSize(u32(w), u32(h));

	double frameTime = 0.0;

	while (glfwWindowShouldClose(g_window) == 0)
	{
		double time1 = glfwGetTime();
		
		g_view->Command_PreDraw();

		g_view->Command_Draw();

		g_debugDraw->DrawString(b3Color_yellow, "%.2f [ms]", 1000.0 * frameTime);

		g_model->Command_Step();

		g_view->Command_PostDraw();

		double time2 = glfwGetTime();
		
		double fraction = 0.9;
		frameTime = fraction * frameTime + (1.0 - fraction) * (time2 - time1);

		glfwSwapBuffers(g_window);
		glfwPollEvents();
	}
}

int main(int argc, char** args)
{
#if defined(_WIN32)
	// Report memory leaks
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
	//_CrtSetBreakAlloc(x);
#endif

	if (glfwInit() == 0)
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}

	// Create window
	g_window = glfwCreateWindow(1024, 768, "Bounce Testbed", NULL, NULL);
	if (g_window == NULL)
	{
		fprintf(stderr, "Failed to create GLFW window\n");
		glfwTerminate();
		return -1;
	}
	
	glfwSetWindowSizeCallback(g_window, WindowSize);
	glfwSetCursorPosCallback(g_window, CursorMove);
	glfwSetScrollCallback(g_window, WheelScroll);
	glfwSetMouseButtonCallback(g_window, MouseButton);
	glfwSetKeyCallback(g_window, KeyButton);
	glfwSwapInterval(1);

	glfwMakeContextCurrent(g_window);

	if (gladLoadGL() == 0)
	{
		fprintf(stderr, "Failed to load OpenGL extensions\n");
		fprintf(stderr, "Error: %d\n", glad_glGetError());
		glfwTerminate();
		return -1;
	}

	printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));

	// 
	g_model = new Model();
	g_view = new View(g_window, g_model);
	g_controller = new Controller(g_model, g_view);

	// Run
	Run();

	//
	delete g_controller;
	g_controller = nullptr;

	delete g_view;
	g_view = nullptr;

	delete g_model;
	g_model = nullptr;

	// 
	glfwTerminate();
	g_window = nullptr;

	return 0;
}