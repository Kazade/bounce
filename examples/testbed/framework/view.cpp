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

#include <testbed/framework/view.h>
#include <testbed/framework/model.h>

#if defined (U_OPENGL_2)
#include <imgui/imgui_impl_glfw_gl2.h>
#elif defined (U_OPENGL_4)
#include <imgui/imgui_impl_glfw_gl3.h>
#else

#endif

static bool GetTestName(void* userData, int idx, const char** name)
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

	// error

#endif

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

View::View(GLFWwindow* window, Model* model)
{
	m_window = window;
	m_model = model;

	// Create UI
	ImGui::CreateContext();

	ImGuiIO& io = ImGui::GetIO();

	io.IniFilename = NULL;
	io.Fonts[0].AddFontDefault();

	ImGui_GLFW_GL_Init(m_window, false);

	ImGui::StyleColorsDark();
}

View::~View()
{
	// Destroy UI
	ImGui_GLFW_GL_Shutdown();

	ImGui::DestroyContext();
}

void View::Command_PreDraw()
{
	ImGui_GLFW_GL_NewFrame();
}

void View::Command_Draw()
{
	Camera& camera = m_model->m_camera;
	Settings& settings = m_model->m_settings;

	ImVec2 buttonSize(-1.0f, 0.0f);

	ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);

	bool openAbout = false;
	if (ImGui::BeginMainMenuBar())
	{

		if (ImGui::BeginMenu("File"))
		{
			if (ImGui::MenuItem("Save"))
			{
				m_model->Action_SaveTest();
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
			ImGui::MenuItem("Reference Grid", "", &settings.drawGrid);
			ImGui::MenuItem("Vertices and Edges", "", &settings.drawVerticesEdges);
			ImGui::MenuItem("Faces", "", &settings.drawFaces);

			ImGui::Separator();

			ImGui::MenuItem("Center of Masses", "", &settings.drawCenterOfMasses);
			ImGui::MenuItem("Bounding Boxes", "", &settings.drawBounds);
			ImGui::MenuItem("Joints", "", &settings.drawJoints);
			ImGui::MenuItem("Contact Points", "", &settings.drawContactPoints);
			ImGui::MenuItem("Contact Normals", "", &settings.drawContactNormals);
			ImGui::MenuItem("Contact Tangents", "", &settings.drawContactTangents);
			ImGui::MenuItem("Contact Polygons", "", &settings.drawContactPolygons);

			ImGui::Separator();

			ImGui::MenuItem("Statistics", "", &settings.drawStats);
			ImGui::MenuItem("Profile", "", &settings.drawProfile);

			ImGui::EndMenu();
		}
		
		if (ImGui::BeginMenu("Tools"))
		{
			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Help"))
		{
			if (ImGui::MenuItem("About"))
			{
				openAbout = true;
			}

			ImGui::EndMenu();
		}

		ImGui::EndMainMenuBar();
	}

	if (openAbout)
	{
		ImGui::OpenPopup("About Bounce Testbed");
		openAbout = false;
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

	ImGui::SetNextWindowPos(ImVec2(camera.m_width - 250.0f, 0.0f));
	ImGui::SetNextWindowSize(ImVec2(250.0f, camera.m_height));

	ImGui::Begin("Test", NULL, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

	ImGui::PushItemWidth(-1.0f);
	
	if (ImGui::Combo("##Test", &settings.testID, GetTestName, NULL, g_testCount, g_testCount))
	{
		m_model->Action_SelectTest(settings.testID);
	}

	ImGui::Separator();

	if (ImGui::Button("Restart", buttonSize))
	{
		m_model->Action_RestartTest();
	}

	if (ImGui::Button("Previous", buttonSize))
	{
		m_model->Action_PreviousTest();
	}

	if (ImGui::Button("Next", buttonSize))
	{
		m_model->Action_NextTest();
	}

	ImGui::Separator();
	
	if (ImGui::Button("Play/Pause", buttonSize))
	{
		m_model->Action_PlayPause();
	}

	if (ImGui::Button("Single Step", buttonSize))
	{
		m_model->Action_SingleStep();
	}

	ImGui::Separator();

	ImGui::Text("Camera");

	ImGui::Separator();
	
	if (ImGui::Button("Restart##Camera", buttonSize))
	{
		m_model->Action_DefaultCamera();
	}

	ImGui::Separator();

	ImGui::Text("Settings");

	ImGui::Separator();

	ImGui::Text("Hertz");
	ImGui::SliderFloat("##Hertz", &settings.hertz, 0.0f, 240.0f, "%.1f");
	ImGui::Text("Velocity Iterations");
	ImGui::SliderInt("##Velocity Iterations", &settings.velocityIterations, 0, 50);
	ImGui::Text("Position Iterations");
	ImGui::SliderInt("#Position Iterations", &settings.positionIterations, 0, 50);
	ImGui::Checkbox("Sleep", &settings.sleep);
	ImGui::Checkbox("Convex Cache", &settings.convexCache);
	ImGui::Checkbox("Warm Start", &settings.warmStart);

	ImGui::PopItemWidth();
	
	ImGui::End();

	ImGui::PopStyleVar();
}

void View::Command_PostDraw()
{
	ImGui::Render();

	ImGui_GLFW_GL_RenderDrawData(ImGui::GetDrawData());
}