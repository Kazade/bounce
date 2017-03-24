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

#if defined(__APPLE_CC__)
#include <OpenGL/gl3.h>
#else
#include <glad/glad.h>
#endif

#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw_gl3.h>
#include <testbed/tests/test.h>

#include <glfw/glfw3.h>

GLFWwindow* g_window;
Settings g_settings;
Test* g_test;
u32 g_testCount;
Camera g_camera;
DebugDraw* g_debugDraw;
Profiler* g_profiler;
bool g_leftDown;
bool g_rightDown;
bool g_shiftDown;
b3Vec2 g_ps0;

static void WindowSize(int w, int h)
{
	g_camera.m_width = float32(w);
	g_camera.m_height = float32(h);
}

static void MouseMove(GLFWwindow* w, double x, double y)
{
	b3Vec2 ps;
	ps.Set(float32(x), float32(y));

	b3Vec2 dp = ps - g_ps0;
	g_ps0 = ps;

	Ray3 pw = g_camera.ConvertScreenToWorld(ps);

	float32 nx = b3Clamp(dp.x, -1.0f, 1.0f);
	float32 ny = b3Clamp(dp.y, -1.0f, 1.0f);

	if (g_shiftDown)
	{
		if (g_leftDown)
		{
			// Negate angles to do positive rotations (CCW) of the world.
			float32 angleX = 0.005f * B3_PI * -nx;
			float32 angleY = 0.005f * B3_PI * -ny;

			b3Quat qx(b3Vec3(1.0f, 0.0f, 0.0f), angleY);
			b3Quat qy(b3Vec3(0.0f, 1.0f, 0.0f), angleX);

			g_camera.m_q = qy * g_camera.m_q;
			g_camera.m_q = g_camera.m_q * qx;
			g_camera.m_q.Normalize();
		}

		if (g_rightDown)
		{
			b3Transform xf = g_camera.BuildWorldTransform();
			g_camera.m_center += 0.2f * nx * xf.rotation.x;
			g_camera.m_center += 0.2f * -ny * xf.rotation.y;
		}
	}
	else
	{
		g_test->MouseMove(pw);
	}
}

static void MouseWheel(GLFWwindow* w, double dx, double dy)
{
	float32 n = b3Clamp(float32(dy), -1.0f, 1.0f);
	if (g_shiftDown)
	{
		g_camera.m_zoom += 0.5f * -n;
	}
}

static void MouseButton(GLFWwindow* w, int button, int action, int mods)
{
	double x, y;
	glfwGetCursorPos(w, &x, &y);
	b3Vec2 p;
	p.Set(float32(x), float32(y));

	Ray3 pw = g_camera.ConvertScreenToWorld(p);

	switch (action)
	{
	case GLFW_PRESS:
	{
		if (button == GLFW_MOUSE_BUTTON_LEFT)
		{
			g_leftDown = true;

			if (g_shiftDown == false)
			{
				g_test->MouseLeftDown(pw);
			}
		}
		
		if (button == GLFW_MOUSE_BUTTON_RIGHT)
		{
			g_rightDown = true;
		}

		break;
	}
	case GLFW_RELEASE:
	{
		if (button == GLFW_MOUSE_BUTTON_LEFT)
		{
			g_leftDown = false;

			if (g_shiftDown == false)
			{
				g_test->MouseLeftUp(pw);
			}
		}

		if (button == GLFW_MOUSE_BUTTON_RIGHT)
		{
			g_rightDown = false;
		}
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
		if (button == GLFW_KEY_LEFT_SHIFT)
		{
			g_shiftDown = true;
			g_test->KeyDown(button);
		}

		if (g_shiftDown)
		{
			if (button == GLFW_KEY_DOWN)
			{
				g_camera.m_zoom += 0.2f;
			}

			if (button == GLFW_KEY_UP)
			{
				g_camera.m_zoom -= 0.2f;
			}
		}
		else
		{
			g_test->KeyDown(button);
		}
		
		break;
	}
	case GLFW_RELEASE:
	{
		if (button == GLFW_KEY_LEFT_SHIFT)
		{
			g_shiftDown = false;
		}

		if (g_shiftDown == false)
		{
			g_test->KeyUp(button);
		}

		break;
	}
	default:
	{
		break;
	}
	}
}

static void Char(GLFWwindow* w, unsigned int codepoint)
{
	ImGui_ImplGlfwGL3_CharCallback(w, codepoint);
}

static void CreateInterface()
{
	ImGui_ImplGlfwGL3_Init(g_window, false);
	ImGuiIO& io = ImGui::GetIO();
	io.Fonts[0].AddFontDefault();
}

static void DestroyInterface()
{
	ImGui_ImplGlfwGL3_Shutdown();
}

static bool GetTestName(void*, int idx, const char** name)
{
	*name = g_tests[idx].name;
	return true;
}

static void Interface()
{
	ImGui::SetNextWindowPos(ImVec2(g_camera.m_width, 0.0f));
	ImGui::SetNextWindowSize(ImVec2(250.0f, g_camera.m_height));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
	
	ImGui::Begin("Controls", NULL, ImVec2(0.0f, 0.0f), 0.25f, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

	ImGui::PushItemWidth(-1.0f);

	ImGui::Text("Test");
	if (ImGui::Combo("##Test", &g_settings.testID, GetTestName, NULL, g_testCount, g_testCount))
	{
		delete g_test;
		g_test = g_tests[g_settings.testID].create();
		g_settings.lastTestID = -1;
	}

	ImVec2 buttonSize = ImVec2(-1, 0);
	if (ImGui::Button("Restart", buttonSize))
	{
		g_settings.lastTestID = -1;
	}
	if (ImGui::Button("Previous", buttonSize))
	{
		g_settings.testID = b3Clamp(g_settings.testID - 1, 0, int(g_testCount) - 1);
		g_settings.lastTestID = -1;
	}
	if (ImGui::Button("Next", buttonSize))
	{
		g_settings.testID = b3Clamp(g_settings.testID + 1, 0, int(g_testCount) - 1);
		g_settings.lastTestID = -1;
	}
	if (ImGui::Button("Dump", buttonSize))
	{
		if (g_test)
		{
			g_test->Dump();
		}
	}
	if (ImGui::Button("Exit", buttonSize))
	{
		glfwSetWindowShouldClose(g_window, true);
	}
	
	ImGui::Separator();

	ImGui::Text("Step");

	ImGui::Text("Hertz");
	ImGui::SliderFloat("##Hertz", &g_settings.hertz, 0.0f, 240.0f, "%.1f");
	ImGui::Text("Velocity Iterations");
	ImGui::SliderInt("##Velocity Iterations", &g_settings.velocityIterations, 0, 50);
	ImGui::Text("Position Iterations");
	ImGui::SliderInt("#Position Iterations", &g_settings.positionIterations, 0, 50);
	ImGui::Checkbox("Sleep", &g_settings.sleep);
	ImGui::Checkbox("Convex Cache", &g_settings.convexCache);
	ImGui::Checkbox("Warm Start", &g_settings.warmStart);

	if (ImGui::Button("Play/Pause", buttonSize))
	{
		g_settings.pause = !g_settings.pause;
	}
	if (ImGui::Button("Single Step", buttonSize))
	{
		g_settings.pause = true;
		g_settings.singleStep = true;
	}
	
	ImGui::Separator();

	ImGui::Text("View");
	ImGui::Checkbox("Grid", &g_settings.drawGrid);
	ImGui::Checkbox("Vertices and Edges", &g_settings.drawVerticesEdges);
	ImGui::Checkbox("Faces", &g_settings.drawFaces);
	ImGui::Checkbox("Center of Masses", &g_settings.drawCenterOfMasses);
	ImGui::Checkbox("Bounding Boxes", &g_settings.drawBounds);
	ImGui::Checkbox("Joints", &g_settings.drawJoints);
	ImGui::Checkbox("Contact Points", &g_settings.drawContactPoints);
	ImGui::Checkbox("Contact Normals", &g_settings.drawContactNormals);
	ImGui::Checkbox("Contact Tangents", &g_settings.drawContactTangents);
	ImGui::Checkbox("Contact Areas", &g_settings.drawContactAreas);
	ImGui::Checkbox("Statistics", &g_settings.drawStats);
	ImGui::Checkbox("Profile", &g_settings.drawProfile);

	ImGui::End();
	ImGui::PopStyleVar();
}

static void Step()
{
	if (g_settings.drawGrid)
	{
		b3Color color(0.2f, 0.2f, 0.2f, 1.0f);
		
		b3Vec3 pn(0.0f, 1.0f, 0.0f);
		b3Vec3 p(0.0f, 0.0f, 0.0f);
		g_debugDraw->DrawCircle(pn, p, 1.0f, color);

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
				vs[0] = b3Vec3((float)i, 0.0f, (float)j);
				vs[1] = b3Vec3((float)i, 0.0f, (float)j + 1);
				vs[2] = b3Vec3((float)i + 1, 0.0f, (float)j + 1);
				vs[3] = b3Vec3((float)i + 1, 0.0f, (float)j);

				for (u32 k = 0; k < 4; ++k)
				{
					vs[k] += t;
				}

				g_debugDraw->DrawPolygon(vs, 4, color);
			}
		}
	}

	if (g_settings.testID != g_settings.lastTestID)
	{
		delete g_test;
		g_settings.lastTestID = g_settings.testID;
		g_test = g_tests[g_settings.testID].create();
		g_settings.pause = true;
	}
	
	g_test->Step();	
	g_debugDraw->Submit();
}

static void Run()
{
	glFrontFace(GL_CCW);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
	glClearDepth(1.0f);

	double t1 = glfwGetTime();
	double frameTime = 0.0;

	while (glfwWindowShouldClose(g_window) == 0)
	{
		int width, height;
		glfwGetWindowSize(g_window, &width, &height);
		g_camera.m_width = float32(width) - 250.0f;
		g_camera.m_height = float32(height);

		glViewport(0, 0, width - 250, height);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		
		ImGui_ImplGlfwGL3_NewFrame();
		
		ImGui::SetNextWindowPos(ImVec2(0, 0));
		ImGui::SetNextWindowSize(ImVec2((float)g_camera.m_width, (float)g_camera.m_height));
		ImGui::Begin("Overlay", NULL, ImVec2(0, 0), 0.0f, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
		ImGui::SetCursorPos(ImVec2(5, (float)g_camera.m_height - 20));
		ImGui::Text("%.1f ms", 1000.0 * frameTime);
		ImGui::End();

		Interface();
		Step();

		double t = glfwGetTime();
		frameTime = t - t1;
		t1 = t;

		ImGui::Render();

		glfwSwapBuffers(g_window);

		glfwPollEvents();
	}
}

int main(int argc, char** args)
{
#if defined(_WIN32)
	// Report memory leaks
	_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
#endif

	if (glfwInit() == 0)
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}
	
	// Create window
	extern b3Version b3_version;
	char title[256];
	sprintf(title, "Bounce Testbed Version %d.%d.%d", b3_version.major, b3_version.minor, b3_version.revision);

#if defined(__APPLE__)
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif
		
	g_window = glfwCreateWindow(1024, 768, title, NULL, NULL);
	if (g_window == NULL)
	{		
		fprintf(stderr, "Failed to open GLFW window\n");
		glfwTerminate();
		return -1;
	}
	
	glfwMakeContextCurrent(g_window);
	glfwSetCursorPosCallback(g_window, MouseMove);
	glfwSetScrollCallback(g_window, MouseWheel);
	glfwSetMouseButtonCallback(g_window, MouseButton);
	glfwSetKeyCallback(g_window, KeyButton);
	glfwSetCharCallback(g_window, Char);
	glfwSwapInterval(1);
	
	if (gladLoadGL() == 0)
	{
		fprintf(stderr, "Failed to load OpenGL extensions\n");
		fprintf(stderr, "Error: %d\n", glad_glGetError());
		glfwTerminate();
		return -1;
	}

	printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));
	
	g_leftDown = false;
	g_rightDown = false;
	g_shiftDown = false;
	g_ps0.SetZero();

	// Create UI
	CreateInterface();

	// Create profiler
	g_profiler = new Profiler();

	// Create renderer
	g_debugDraw = new DebugDraw();

	// Run the testbed
	g_testCount = 0;
	while (g_tests[g_testCount].create != NULL)
	{
		++g_testCount;
	}
	g_test = NULL;

	Run();

	// Destroy the last test
	if (g_test)
	{
		delete g_test;
		g_test = NULL;
	}

	// Destroy renderer
	delete g_debugDraw;

	// Destroy profiler
	delete g_profiler;

	// Destroy UI
	DestroyInterface();

	// Destroy g_window
	glfwTerminate();
	
	return 0;
}