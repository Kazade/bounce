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

#include <testbed/framework/view.h>
#include <testbed/framework/view_model.h>
#include <testbed/framework/test.h>
#include <testbed/framework/profiler.h>
#include <testbed/framework/profiler_st.h>

#include <imgui/imgui.h>
#if defined (U_OPENGL_2)
#include <imgui/imgui_impl_glfw_gl2.h>
#elif defined (U_OPENGL_4)
#include <imgui/imgui_impl_glfw_gl3.h>
#else
#endif

#include <glfw/glfw3.h>

static inline bool GetTestName(void* userData, int idx, const char** name)
{
	assert(u32(idx) < g_testCount);
	*name = g_tests[idx].name;
	return true;
}

static inline bool ImGui_GLFW_GL_Init(GLFWwindow* w, bool install_callbacks)
{

#if defined(U_OPENGL_2)

	return ImGui_ImplGlfwGL2_Init(w, install_callbacks);

#elif defined(U_OPENGL_4)

	return ImGui_ImplGlfwGL3_Init(w, install_callbacks);

#else

#endif
	return false;
}

static inline void ImGui_GLFW_GL_Shutdown()
{

#if defined(U_OPENGL_2)

	ImGui_ImplGlfwGL2_Shutdown();

#elif defined(U_OPENGL_4)

	ImGui_ImplGlfwGL3_Shutdown();

#else

	// error

#endif

}

static inline void ImGui_GLFW_GL_NewFrame()
{

#if defined(U_OPENGL_2)

	ImGui_ImplGlfwGL2_NewFrame();

#elif defined(U_OPENGL_4)

	ImGui_ImplGlfwGL3_NewFrame();

#else

	// error

#endif

}

static inline void ImGui_GLFW_GL_RenderDrawData(ImDrawData* draw_data)
{

#if defined(U_OPENGL_2)

	ImGui_ImplGlfwGL2_RenderDrawData(draw_data);

#elif defined(U_OPENGL_4)

	ImGui_ImplGlfwGL3_RenderDrawData(draw_data);

#else

	// error

#endif

}

View::View(GLFWwindow* window)
{
	m_viewModel = nullptr;
	m_window = window;

	// Create UI
	ImGui::CreateContext();

	ImGuiIO& io = ImGui::GetIO();

	io.IniFilename = NULL;
	io.Fonts[0].AddFontDefault();

	ImGui_GLFW_GL_Init(m_window, false);

	ImGui::StyleColorsDark();

	m_ps0.SetZero();
}

View::~View()
{
	// Destroy UI
	ImGui_GLFW_GL_Shutdown();

	ImGui::DestroyContext();
}

b3Vec2 View::GetCursorPosition() const
{
	double x, y;
	glfwGetCursorPos(m_window, &x, &y);
	return b3Vec2(float32(x), float32(y));
}

void View::Event_SetWindowSize(int w, int h)
{
	m_viewModel->Event_SetWindowSize(w, h);
}

void View::Event_Press_Key(int button)
{
	m_viewModel->Event_Press_Key(button);
}

void View::Event_Release_Key(int button)
{
	m_viewModel->Event_Release_Key(button);
}

void View::Event_Press_Mouse(int button)
{
	m_viewModel->Event_Press_Mouse(button);
}

void View::Event_Release_Mouse(int button)
{
	m_viewModel->Event_Release_Mouse(button);
}

void View::Event_Move_Cursor(float x, float y)
{
	m_viewModel->Event_Move_Cursor(x, y);
	m_ps0.Set(x, y);
}

void View::Event_Scroll(float dx, float dy)
{
	m_viewModel->Event_Scroll(dx, dy);
}

void View::BeginInterface()
{
	ImGui_GLFW_GL_NewFrame();

	ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
}

void View::Interface()
{
	Settings& settings = m_viewModel->m_settings;
	TestSettings& testSettings = m_viewModel->m_testSettings;

	bool openControls = false;
	bool openAbout = false;
	if (ImGui::BeginMainMenuBar())
	{
		if (ImGui::BeginMenu("File"))
		{
			if (ImGui::MenuItem("Save"))
			{
				m_viewModel->Action_SaveTest();
			}

			ImGui::Separator();

			if (ImGui::MenuItem("Exit", "Alt+F4"))
			{
				glfwSetWindowShouldClose(m_window, true);
			}

			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("View"))
		{
			ImGui::MenuItem("Profile Tree", "", &settings.drawProfileTree);
			ImGui::MenuItem("Profile Tree Statistics", "", &settings.drawProfileTreeStats);
			ImGui::MenuItem("Statistics", "", &settings.drawStats);

			ImGui::Separator();

			ImGui::MenuItem("Points", "", &settings.drawPoints);
			ImGui::MenuItem("Lines", "", &settings.drawLines);
			ImGui::MenuItem("Triangles", "", &settings.drawTriangles);

			ImGui::Separator();
			
			ImGui::MenuItem("Reference Grid", "", &settings.drawGrid);

			ImGui::Separator();

			ImGui::MenuItem("Center of Masses", "", &testSettings.drawCenterOfMasses);
			ImGui::MenuItem("Bounding Boxes", "", &testSettings.drawBounds);
			ImGui::MenuItem("Shapes", "", &testSettings.drawShapes);
			ImGui::MenuItem("Joints", "", &testSettings.drawJoints);
			ImGui::MenuItem("Contact Points", "", &testSettings.drawContactPoints);
			ImGui::MenuItem("Contact Normals", "", &testSettings.drawContactNormals);
			ImGui::MenuItem("Contact Tangents", "", &testSettings.drawContactTangents);
			ImGui::MenuItem("Contact Polygons", "", &testSettings.drawContactPolygons);

			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Tools"))
		{
			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Help"))
		{
			if (ImGui::MenuItem("Controls"))
			{
				openControls = true;
			}
			
			if (ImGui::MenuItem("About"))
			{
				openAbout = true;
			}

			ImGui::EndMenu();
		}

		ImGui::EndMainMenuBar();
	}
	
	if (openControls)
	{
		ImGui::OpenPopup("Controls");
	}

	if (openAbout)
	{
		ImGui::OpenPopup("About Bounce Testbed");
	}

	ImVec2 buttonSize(-1.0f, 0.0f);
	
	if (ImGui::BeginPopupModal("Controls", NULL, ImGuiWindowFlags_Popup | ImGuiWindowFlags_NoResize))
	{
		ImGui::Text("Rotate the scene using LSHIFT + LMB");
		ImGui::Text("Translate the scene using LSHIFT + RMB");
		ImGui::Text("Zoom in / out the scene using LSHIFT + Mouse Wheel");

		if (ImGui::Button("OK", buttonSize))
		{
			ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}

	if (ImGui::BeginPopupModal("About Bounce Testbed", NULL, ImGuiWindowFlags_Popup | ImGuiWindowFlags_NoResize))
	{
		extern b3Version b3_version;

		ImGui::Text("Bounce Testbed");
		ImGui::Text("Version %d.%d.%d", b3_version.major, b3_version.minor, b3_version.revision);
		ImGui::Text("Copyright (c) Irlan Robson");
		ImGui::Text("https://github.com/irlanrobson/bounce");

		if (ImGui::Button("OK", buttonSize))
		{
			ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}

	ImGui::SetNextWindowPos(ImVec2(0.0f, 20.0f));
	ImGui::SetNextWindowSize(ImVec2(g_camera->m_width, 20.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowMinSize, ImVec2(0.0f, 0.0f));

	ImGui::Begin("##ToolBar", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_MenuBar);

	if (ImGui::BeginMenuBar())
	{
		ImGui::PushItemWidth(250.0f);

		ImGui::Separator();
		
		if (ImGui::Combo("##Test", &settings.testID, GetTestName, NULL, g_testCount, g_testCount))
		{
			m_viewModel->Action_SetTest();
		}

		ImGui::PopItemWidth();

		ImVec2 menuButtonSize(100.0f, 0.0f);

		ImGui::Separator();

		if (ImGui::Button("Previous", menuButtonSize))
		{
			m_viewModel->Action_PreviousTest();
		}

		if (ImGui::Button("Next", menuButtonSize))
		{
			m_viewModel->Action_NextTest();
		}

		ImGui::Separator();

		if (ImGui::Button("Play/Pause", menuButtonSize))
		{
			m_viewModel->Action_PlayPause();
		}

		if (ImGui::Button("Single Play", menuButtonSize))
		{
			m_viewModel->Action_SinglePlay();
		}

		ImGui::Separator();

		if (ImGui::Button("Restart", menuButtonSize))
		{
			m_viewModel->Action_SetTest();
		}

		ImGui::Separator();

		if (ImGui::Button("Reset Camera", menuButtonSize))
		{
			m_viewModel->Action_ResetCamera();
		}

		ImGui::EndMenuBar();
	}

	ImGui::End();
	
	ImGui::PopStyleVar();

	ImGui::SetNextWindowPos(ImVec2(g_camera->m_width - 250.0f, 40.0f));
	ImGui::SetNextWindowSize(ImVec2(250.0f, g_camera->m_height - 40.0f));
	ImGui::Begin("Test Settings", NULL, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

	ImGui::PushItemWidth(-1.0f);

	ImGui::Text("Hertz");
	ImGui::SliderFloat("##Hertz", &testSettings.hertz, 0.0f, 240.0f, "%.1f");

	ImGui::Text("Velocity Iterations");
	ImGui::SliderInt("##Velocity Iterations", &testSettings.velocityIterations, 0, 50);

	ImGui::Text("Position Iterations");
	ImGui::SliderInt("##Position Iterations", &testSettings.positionIterations, 0, 50);

	ImGui::Checkbox("Sleep", &testSettings.sleep);
	ImGui::Checkbox("Convex Cache", &testSettings.convexCache);
	ImGui::Checkbox("Warm Start", &testSettings.warmStart);

	ImGui::PopItemWidth();

	ImGui::End();
}

static void TreeNode(ProfilerNode* node, u32& index)
{
	ImGui::PushID(index);
	++index;

	if (ImGui::TreeNode(node->name))
	{
		float64 elapsedTime = node->t1 - node->t0;
		ImGui::Text("%.4f [ms]", elapsedTime);

		for (u32 i = 0; i < node->children.Count(); ++i)
		{
			TreeNode(node->children[i], index);
		}
		ImGui::TreePop();
	}	
	
	ImGui::PopID();
}

void View::InterfaceProfileTree()
{
	ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImVec2 ws = ImGui::GetWindowSize();
	ImVec2 wp = ImGui::GetWindowPos();
	ImGui::End();

	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
	
	ImGui::SetNextWindowBgAlpha(0.0f);
	ImGui::SetNextWindowPos(ImVec2(0.0f, wp.y + ws.y));
	ImGui::SetNextWindowSize(ImVec2(g_camera->m_width - 250.0f, 0.0f));

	ImGui::Begin("Profile Tree", NULL, ImGuiWindowFlags_AlwaysAutoResize);

	ProfilerNode* root = g_profiler->GetRoot();
	if (root)
	{
		u32 index = 0;
		TreeNode(root, index);
	}
	
	ImGui::End();
	
	ImGui::PopStyleVar();
}

static void TreeNode(ProfilerStNode* node, u32& index)
{
	ImGui::PushID(index);
	++index;

	if (ImGui::TreeNode(node->name))
	{
		ImGui::Text("%.4f (min = %.4f) (max = %.4f) (calls = %d) [ms]", node->elapsed, node->stat->minElapsed, node->stat->maxElapsed, node->callCount);

		for (u32 i = 0; i < node->children.Count(); ++i)
		{
			TreeNode(node->children[i], index);
		}
		ImGui::TreePop();
	}

	ImGui::PopID();
}

void View::InterfaceProfileTreeStats()
{
	ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImVec2 wp = ImGui::GetWindowPos();
	ImVec2 ws = ImGui::GetWindowSize();
	ImGui::End();

	wp.y = wp.y + ws.y;

	if (g_settings->drawProfileTree)
	{
		ImGui::Begin("Profile Tree", NULL, ImGuiWindowFlags_AlwaysAutoResize);
		ImVec2 ptwp = ImGui::GetWindowPos();
		ImVec2 ptws = ImGui::GetWindowSize();
		ImGui::End();

		wp.y = ptwp.y + ptws.y;
	}

	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);

	ImGui::SetNextWindowBgAlpha(0.0f);
	ImGui::SetNextWindowPos(ImVec2(0.0f, wp.y));
	ImGui::SetNextWindowSize(ImVec2(g_camera->m_width - 250.0f, 0.0f));

	ImGui::Begin("Profile Tree Statistics", NULL, ImGuiWindowFlags_AlwaysAutoResize);

	ProfilerStNode* root = g_profilerSt->GetRoot();
	if (root)
	{
		u32 index = 0;
		TreeNode(root, index);
	}

	ImGui::End();

	ImGui::PopStyleVar();
}

void View::EndInterface()
{
	ImGui::PopStyleVar();

	ImGui::Render();

	ImGui_GLFW_GL_RenderDrawData(ImGui::GetDrawData());
}