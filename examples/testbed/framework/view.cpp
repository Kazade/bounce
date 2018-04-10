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
	io.Fonts[0].AddFontDefault();

	ImGui_GLFW_GL_Init(m_window, false);

	ImGui::StyleColorsLight();
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

	ImGui::SetNextWindowPos(ImVec2(camera.m_width - 250.0f, 0.0f));
	ImGui::SetNextWindowSize(ImVec2(250.0f, camera.m_height));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);

	ImGui::Begin("Controller", NULL, ImVec2(0.0f, 0.0f), 0.25f, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

	ImGui::PushItemWidth(-1.0f);
	
	ImGui::Separator();
	
	ImGui::Text("Test");

	ImGui::Separator();
	
	if (ImGui::Combo("##Test", &settings.testID, GetTestName, NULL, g_testCount, g_testCount))
	{
		m_model->Action_SelectTest(settings.testID);
	}

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
	
	if (ImGui::Button("Dump", buttonSize))
	{
		m_model->Action_DumpTest();
	}

	if (ImGui::Button("Exit", buttonSize))
	{
		glfwSetWindowShouldClose(m_window, true);
	}

	ImGui::Separator();

	ImGui::Text("Step");

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

	if (ImGui::Button("Play/Pause", buttonSize))
	{
		m_model->Action_PlayPause();
	}
	
	if (ImGui::Button("Single Step", buttonSize))
	{
		m_model->Action_SingleStep();
	}

	ImGui::Separator();

	ImGui::Text("View");
	
	ImGui::Separator();
	
	ImGui::Checkbox("Reference Grid", &settings.drawGrid);
	ImGui::Checkbox("Vertices and Edges", &settings.drawVerticesEdges);
	ImGui::Checkbox("Faces", &settings.drawFaces);
	ImGui::Checkbox("Center of Masses", &settings.drawCenterOfMasses);
	ImGui::Checkbox("Bounding Boxes", &settings.drawBounds);
	ImGui::Checkbox("Joints", &settings.drawJoints);
	ImGui::Checkbox("Contact Points", &settings.drawContactPoints);
	ImGui::Checkbox("Contact Normals", &settings.drawContactNormals);
	ImGui::Checkbox("Contact Tangents", &settings.drawContactTangents);
	ImGui::Checkbox("Contact Polygons", &settings.drawContactPolygons);
	ImGui::Checkbox("Statistics", &settings.drawStats);
	ImGui::Checkbox("Profile", &settings.drawProfile);
	
	ImGui::Separator();

	if (ImGui::Button("Left", buttonSize))
	{
		m_model->Action_LeftCamera();
	}

	if (ImGui::Button("Right", buttonSize))
	{
		m_model->Action_RightCamera();
	}

	if (ImGui::Button("Bottom", buttonSize))
	{
		m_model->Action_BottomCamera();
	}

	if (ImGui::Button("Top", buttonSize))
	{
		m_model->Action_TopCamera();
	}

	if (ImGui::Button("Back", buttonSize))
	{
		m_model->Action_BackCamera();
	}

	if (ImGui::Button("Front", buttonSize))
	{
		m_model->Action_FrontCamera();
	}

	ImGui::End();
	ImGui::PopStyleVar();
}

void View::Command_PostDraw()
{
	ImGui::Render();
	
	ImGui_GLFW_GL_RenderDrawData(ImGui::GetDrawData());
}