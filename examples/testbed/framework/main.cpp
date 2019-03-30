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

#if defined (U_OPENGL_2)
	#include <glad_2/glad.h>
#elif defined (U_OPENGL_4)
	#include <glad_4/glad.h>
#else
	// error
#endif

#include <glfw/glfw3.h>

#include <testbed/framework/model.h>
#include <testbed/framework/view.h>
#include <testbed/framework/view_model.h>

//
GLFWwindow* g_window;

//
static Model* g_model;
static View* g_view;
static ViewModel* g_viewModel;

static void WindowSize(GLFWwindow* ww, int w, int h)
{
	g_view->Event_SetWindowSize(w, h);
}

static void CursorMove(GLFWwindow* w, double x, double y)
{
	g_view->Event_Move_Cursor(float(x), float(y));
}

static void WheelScroll(GLFWwindow* w, double dx, double dy)
{
	g_view->Event_Scroll(float(dx), float(dy));
}

static void MouseButton(GLFWwindow* w, int button, int action, int mods)
{
	switch (action)
	{
	case GLFW_PRESS:
	{
		g_view->Event_Press_Mouse(button);
		break;
	}
	case GLFW_RELEASE:
	{
		g_view->Event_Release_Mouse(button);
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
		g_view->Event_Press_Key(button);
		break;
	}
	case GLFW_RELEASE:
	{
		g_view->Event_Release_Key(button);
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
	g_view->Event_SetWindowSize(u32(w), u32(h));
	
	while (glfwWindowShouldClose(g_window) == 0)
	{
		g_profiler->Begin();

		g_profiler->PushEvent("Frame");
		
		g_view->BeginInterface();

		if (g_model->IsPaused())
		{
			g_draw->DrawString(b3Color_white, "*PAUSED*");
		}
		else
		{
			g_draw->DrawString(b3Color_white, "*PLAYING*");
		}

		if (g_settings->drawProfile)
		{
			const b3Array<ProfilerRecord>& records = g_recorderProfiler->GetRecords();
			for (u32 i = 0; i < records.Count(); ++i)
			{
				const ProfilerRecord& r = records[i];
				if (r.call != 0)
				{
					g_draw->DrawString(b3Color_white, "%s %.4f (%.4f) [ms]", r.name, r.elapsed, r.maxElapsed);
				}
			}
		}

		g_view->Interface();

		g_model->Update();

		g_view->EndInterface();

		g_profiler->PopEvent();
		
		g_profiler->End();

		glfwSwapBuffers(g_window);
		glfwPollEvents();
	}
}

int main(int argc, char** args)
{
#if defined(_WIN32)
	// Report memory leaks
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
	//_CrtSetBreakAlloc(0);
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
	g_view = new View(g_window);
	g_viewModel = new ViewModel(g_model, g_view);

	// Run
	Run();

	//
	delete g_viewModel;
	g_viewModel = nullptr;

	delete g_view;
	g_view = nullptr;

	delete g_model;
	g_model = nullptr;

	// 
	glfwTerminate();
	g_window = nullptr;

	return 0;
}