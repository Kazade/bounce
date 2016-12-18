set(imgui_HEADERS 
	imconfig.h 
	imgui.h
	imgui_internal.h
	imgui_impl_glfw_gl3.h
	stb_rect_pack.h
	stb_textedit.h
	stb_truetype.h	
)

set(imgui_SOURCES
	imgui.cpp
	imgui_draw.cpp
	imgui_impl_glfw_gl3.cpp
)

include_directories(../)
add_library(imgui STATIC ${imgui_HEADERS} ${imgui_SOURCES})