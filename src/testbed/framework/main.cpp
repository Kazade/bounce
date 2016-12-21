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

#include <glad/glad.h>
#include <glfw/glfw3.h>
#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw_gl3.h>
#include <testbed/tests/test.h>

GLFWwindow* g_window;
Settings g_settings;
Test* g_test;
Camera g_camera;
DebugDraw* g_debugDraw;
bool g_leftDown;
bool g_rightDown;
bool g_shiftDown;
b3Vec2 g_ps0;

void WindowSize(int w, int h)
{
	g_camera.m_width = float32(w);
	g_camera.m_height = float32(h);
}

void MouseMove(GLFWwindow* w, double x, double y)
{
	b3Vec2 p;
	p.Set(float32(x), float32(y));

	b3Vec2 dp = p - g_ps0;
	g_ps0 = p;

	Ray3 pw = g_camera.ConvertScreenToWorld(p);

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

void MouseWheel(GLFWwindow* w, double dx, double dy)
{
	float32 n = b3Clamp(float32(dy), -1.0f, 1.0f);
	if (g_shiftDown)
	{
		g_camera.m_zoom += 0.5f * -n;
	}
}

void MouseButton(GLFWwindow* w, int button, int action, int mods)
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

void KeyButton(GLFWwindow* w, int button, int scancode, int action, int mods)
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
				g_camera.m_zoom += 0.05f;
			}

			if (button == GLFW_KEY_UP)
			{
				g_camera.m_zoom -= 0.05f;
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

void Char(GLFWwindow* w, unsigned int codepoint)
{
	ImGui_ImplGlfwGL3_CharCallback(w, codepoint);
}

void CreateInterface()
{
	ImGui_ImplGlfwGL3_Init(g_window, false);
	ImGuiIO& io = ImGui::GetIO();
	io.Fonts[0].AddFontDefault();

	ImGuiStyle& style = ImGui::GetStyle();
	style.FrameRounding = style.GrabRounding = style.ScrollbarRounding = 2.0f;
	style.FramePadding = ImVec2(4, 2);
	style.DisplayWindowPadding = ImVec2(0, 0);
	style.DisplaySafeAreaPadding = ImVec2(0, 0);
}

void DestroyInterface()
{
	ImGui_ImplGlfwGL3_Shutdown();
}

bool GetTestName(void*, int idx, const char** name)
{
	*name = g_tests[idx].name;
	return true;
}

void Interface()
{
	ImGui::SetNextWindowPos(ImVec2(g_camera.m_width - 250.0f, 0.0f));
	ImGui::SetNextWindowSize(ImVec2(250.0f, g_camera.m_height));
	ImGui::Begin("Controls", NULL, ImVec2(0.0f, 0.0f), 0.25f, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

	ImGui::PushItemWidth(-1.0f);

	ImGui::Text("Test");
	if (ImGui::Combo("##Test", &g_settings.testID, GetTestName, NULL, e_testCount, e_testCount))
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
		g_settings.testID = b3Clamp(g_settings.testID - 1, 0, int(e_testCount) - 1);
		g_settings.lastTestID = -1;
	}
	if (ImGui::Button("Next", buttonSize))
	{
		g_settings.testID = b3Clamp(g_settings.testID + 1, 0, int(e_testCount) - 1);
		g_settings.lastTestID = -1;
	}
	if (ImGui::Button("Exit", buttonSize))
	{
		glfwSetWindowShouldClose(g_window, true);
	}
	
	ImGui::Separator();

	ImGui::Text("Step");
	
	ImGui::Text("Hertz");
	ImGui::SliderFloat("##Hertz", &g_settings.hertz, 0.0f, 240.0f, "%.4f");
	ImGui::Text("Velocity Iterations");
	ImGui::SliderInt("##Velocity Iterations", &g_settings.velocityIterations, 0, 50);
	ImGui::Text("Position Iterations");
	ImGui::SliderInt("#Position Iterations", &g_settings.positionIterations, 0, 50);
	ImGui::Checkbox("Warm Start", &g_settings.warmStart);
	ImGui::Checkbox("Sleep", &g_settings.sleep);
	//ImGui::Checkbox("Convex Cache", &g_settings.convexCache);

	if (ImGui::Button("Play/Pause", buttonSize))
	{
		g_settings.pause = !g_settings.pause;
	}
	if (ImGui::Button("Single Step", buttonSize))
	{
		g_settings.pause = true;
		g_settings.singleStep = true;
	}

	ImGui::PopItemWidth();

	ImGui::Separator();

	ImGui::Text("View");
	ImGui::Checkbox("Grid", &g_settings.drawGrid);
	ImGui::Checkbox("Polygons", &g_settings.drawShapes);
	ImGui::Checkbox("Center of Masses", &g_settings.drawCenterOfMasses);
	ImGui::Checkbox("Bounding Boxes", &g_settings.drawBounds);
	ImGui::Checkbox("Joints", &g_settings.drawJoints);
	ImGui::Checkbox("Contact Points", &g_settings.drawContactPoints);
	ImGui::Checkbox("Contact Normals", &g_settings.drawContactNormals);
	ImGui::Checkbox("Contact Tangents", &g_settings.drawContactTangents);
	ImGui::Checkbox("Statistics", &g_settings.drawStats);
	ImGui::Checkbox("Profile", &g_settings.drawProfile);
	
	ImGui::End();
}

void Step()
{
	if (g_settings.testID != g_settings.lastTestID)
	{
		delete g_test;
		g_settings.lastTestID = g_settings.testID;
		g_test = g_tests[g_settings.testID].create();
		g_settings.pause = true;
	}
	g_test->Step();
}

void Run()
{
	glFrontFace(GL_CCW);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
	glClearDepth(1.0f);

	while (glfwWindowShouldClose(g_window) == 0)
	{
		int width, height;
		glfwGetWindowSize(g_window, &width, &height);
		g_camera.m_width = float32(width);
		g_camera.m_height = float32(height);

		glViewport(0, 0, width, height);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		
		ImGui_ImplGlfwGL3_NewFrame();
		
		if (g_settings.drawGrid)
		{
			u32 n = 20;

			b3Vec3 t;
			t.x = -0.5f * float32(n);
			t.y = 0.0f;
			t.z = -0.5f * float32(n);

			b3Color color(1.0f, 1.0f, 1.0f, 1.0f);
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

			b3Color color2(0.0f, 0.0f, 0.0f);

			{
				b3Vec3 p1(t.x, 0.005f, 0.0f);
				b3Vec3 p2(-t.x, 0.005f, 0.0f);

				g_debugDraw->DrawSegment(p1, p2, color2);
			}

			{
				b3Vec3 p1(0.0f, 0.005f, t.x);
				b3Vec3 p2(0.0f, 0.005f, -t.x);

				g_debugDraw->DrawSegment(p1, p2, color2);
			}
		}
		
		Step();
		
		g_debugDraw->Submit();

		if (g_settings.drawShapes)
		{
			g_debugDraw->Submit(g_test->m_world);
		}
		
		Interface();

		ImGui::Render();

		glfwSwapBuffers(g_window);

		glfwPollEvents();
	}
}

int main(int argc, char** args)
{
	if (glfwInit() == 0)
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}
	
	// Create g_window
	extern b3Version b3_version;
	char title[256];
	sprintf(title, "Bounce Testbed Version %d.%d.%d", b3_version.major, b3_version.minor, b3_version.revision);
	
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    	
	g_window = glfwCreateWindow(1024, 768, title, NULL, NULL);
	if (g_window == NULL)
	{		
		fprintf(stderr, "Failed to opengl GLFW window\n");
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
		fprintf(stderr, "Error: %d\n", glad_glGetError());
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));
	
	g_leftDown = false;
	g_rightDown = false;
	g_shiftDown = false;
	g_ps0.SetZero();

	// Create UI
	CreateInterface();

	// Create renderer
	g_debugDraw = new DebugDraw();

	// Run the testbed
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

	// Destroy UI
	DestroyInterface();

	// Destroy g_window
	glfwTerminate();
}
