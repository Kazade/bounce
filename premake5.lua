-- Irlan Robson
-- Bounce premake script
-- http://industriousone.com/premake

-- variable paths for the case files are moved
solution_name = "bounce"
working_dir = "."
solution_dir = "build/"
external_dir = "external/"
bounce_inc_dir = "include/"
bounce_src_dir = "src/"
examples_inc_dir = "examples/"
examples_src_dir = "examples/"
tests_inc_dir = "test/"
tests_src_dir = "test/"
obj_dir = "/obj/"
bin_dir = "/bin/"

-- or "" to make --help work
action = _ACTION or ""

-- list of graphics APIs
newoption 
{
   trigger     = "gfxapi",
   value       = "API",
   description = "Choose a graphics API",
   allowed = 
   {
      { "opengl_2",    "OpenGL 2" },
      { "opengl_4",    "OpenGL 4" }
   }
}

-- defaults to OpenGL 4
if not _OPTIONS["gfxapi"] then
   _OPTIONS["gfxapi"] = "opengl_4"
end

-- premake main
workspace(solution_name)
	configurations { "debug", "release" }
	location(solution_dir .. "/" .. action)
	symbols "On"
	warnings 'Extra'

	filter "system:windows" 
		platforms { "x86", "x86_x64" }
	    defaultplatform "x86_64"
		defines { "_CRT_SECURE_NO_WARNINGS", "_WIN32", "WIN32", "_WINDOWS" }
		
	filter "system:linux" 
		platforms { "x86_64" }
		cppdialect "C++11"
	
	filter {}
	
	filter "configurations:debug"
		defines { "DEBUG" }
		optimize "Off"
		targetdir ( solution_dir .. action .. bin_dir .. "%{cfg.platform}/%{cfg.buildcfg}/%{prj.name}" )
		objdir ( "!" .. solution_dir .. action .. obj_dir .. "%{cfg.platform}/%{cfg.buildcfg}/%{prj.name}" )	
    
	filter "configurations:release"
		defines { "NDEBUG" }
		optimize "On"
		targetdir ( solution_dir .. action .. bin_dir .. "%{cfg.platform}/%{cfg.buildcfg}/%{prj.name}" )
		objdir ( "!" .. solution_dir .. action .. obj_dir .. "%{cfg.platform}/%{cfg.buildcfg}/%{prj.name}" )
	
	filter {}
	
	filter "options:gfxapi=opengl_2" 
		defines { "U_OPENGL_2" }
	
	filter "options:gfxapi=opengl_4" 
		defines { "U_OPENGL_4" }
	
	filter {}
		
	project "bounce"
		kind "StaticLib"
		location ( solution_dir .. action )
		includedirs { bounce_inc_dir, external_dir }
		vpaths { [""] = "bounce" }

		files 
		{ 
			bounce_inc_dir .. "/bounce/**.h", 
			bounce_inc_dir .. "/bounce/**.inl", 
			bounce_src_dir .. "/bounce/**.cpp" 
		}
	project "glad"
		kind "StaticLib"
		language "C"
		location ( solution_dir .. action )
		includedirs { external_dir }
		vpaths { ["Headers"] = "**.h", ["Sources"] = "**.c" }	

		filter { "system:windows", "options:gfxapi=opengl_2" } 
			files 
			{ 
				external_dir .. "/glad_2/khrplatform.h",
				external_dir .. "/glad_2/glad.h", 
				external_dir .. "/glad_2/glad.c",
			}
				
		filter { "system:windows", "options:gfxapi=opengl_4" } 
			files 
			{ 
				external_dir .. "/glad_4/khrplatform.h",
				external_dir .. "/glad_4/glad.h", 
				external_dir .. "/glad_4/glad.c",
			}
				
		filter { "system:linux", "options:gfxapi=opengl_2" } 
			files 
			{ 
				external_dir .. "/glad_2/khrplatform.h",
				external_dir .. "/glad_2/glad_glx.h", 
				external_dir .. "/glad_2/glad_glx.c",
			}
				
		filter { "system:linux", "options:gfxapi=opengl_4" }
			files 
			{ 
				external_dir .. "/glad_4/khrplatform.h",
				external_dir .. "/glad_4/glad_glx.h", 
				external_dir .. "/glad_4/glad_glx.c",
			}
				
	project "glfw"
		kind "StaticLib"
		language "C"
		location ( solution_dir .. action )
		includedirs { external_dir .. "/glfw" }
		defines { "_GLFW_USE_CONFIG_H" } -- see glfw_config.h
		vpaths { ["Headers"] = "**.h", ["Sources"] = "**.c" }

		files
		{
			external_dir .. "/glfw/glfw_config.h",
			external_dir .. "/glfw/glfw3.h",
			external_dir .. "/glfw/glfw3native.h",
			
			external_dir .. "/glfw/context.c", 		
			external_dir .. "/glfw/init.c",
			external_dir .. "/glfw/input.c",
			external_dir .. "/glfw/monitor.c",
			external_dir .. "/glfw/vulkan.c",
			external_dir .. "/glfw/window.c",
		}	
		
		filter "system:windows" 
			files 
			{	 
				external_dir .. "/glfw/win32_platform.h",
				external_dir .. "/glfw/win32_joystick.h",
				external_dir .. "/glfw/wgl_context.h",
				external_dir .. "/glfw/egl_context.h",
				
				external_dir .. "/glfw/win32_init.c",
				external_dir .. "/glfw/win32_joystick.c",
				external_dir .. "/glfw/win32_monitor.c",
				external_dir .. "/glfw/win32_time.c",
				external_dir .. "/glfw/win32_tls.c",
				external_dir .. "/glfw/win32_window.c",
				external_dir .. "/glfw/wgl_context.c",
				external_dir .. "/glfw/egl_context.c",				
			}

		filter "system:linux" 
         	buildoptions { "-pthread" }
			files 
			{	 
				external_dir .. "/glfw/x11_platform.h",
				external_dir .. "/glfw/xkb_unicode.h",
				external_dir .. "/glfw/linux_joystick.h",
				external_dir .. "/glfw/posix_time.h",	
				external_dir .. "/glfw/posix_tls.h",		
				external_dir .. "/glfw/glx_context.h",
				external_dir .. "/glfw/egl_context.h",
				
				external_dir .. "/glfw/x11_init.c",	
				external_dir .. "/glfw/x11_monitor.c",
				external_dir .. "/glfw/x11_window.c",
				external_dir .. "/glfw/xkb_unicode.c",
				external_dir .. "/glfw/linux_joystick.c",
				external_dir .. "/glfw/posix_time.c",
				external_dir .. "/glfw/posix_tls.c",
				external_dir .. "/glfw/egl_context.c",
				external_dir .. "/glfw/glx_context.c",					
			}	
			
	project "imgui"
		kind "StaticLib"
		language "C++"
		location ( solution_dir .. action )
		includedirs { external_dir } 
		vpaths { ["Headers"] = "**.h", ["Sources"] = "**.cpp" }		
	
		files 
		{ 
			external_dir .. "/imgui/imconfig.h", 
			external_dir .. "/imgui/imgui.h", 
			external_dir .. "/imgui/imgui_internal.h", 
			
			external_dir .. "/imgui/stb_rect_pack.h", 
			external_dir .. "/imgui/stb_textedit.h", 
			external_dir .. "/imgui/stb_truetype.h", 
			
			external_dir .. "/imgui/imgui.cpp",
			external_dir .. "/imgui/imgui_demo.cpp",
			external_dir .. "/imgui/imgui_draw.cpp" 
		}

		filter "options:gfxapi=opengl_2"
			files 
			{ 
				external_dir .. "/imgui/imgui_impl_glfw_gl2.h", 
				external_dir .. "/imgui/imgui_impl_glfw_gl2.cpp" 
			}
			
		filter "options:gfxapi=opengl_4" 
			files 
			{ 
				external_dir .. "/imgui/imgui_impl_glfw_gl3.h", 
				external_dir .. "/imgui/imgui_impl_glfw_gl3.cpp"
			}
	
	project "rapidjson"
		kind "StaticLib"
		language "C++"
		location ( solution_dir .. action )
		includedirs { external_dir } 
		vpaths { ["Headers"] = "**.h", ["Sources"] = "**.cpp" }		

		files 
		{ 
			external_dir .. "/rapidjson/**.h", 
			external_dir .. "/rapidjson/**.cpp" 
		}
		
	project "triangle"
		kind "StaticLib"
		language "C"
		location ( solution_dir .. action )
		includedirs { external_dir } 
		vpaths { ["Headers"] = "**.h", ["Sources"] = "**.c" }		

		files 
		{ 
			external_dir .. "/triangle/**.h", 
			external_dir .. "/triangle/**.c" 
		}
		
	project "testbed"
		kind "ConsoleApp"
		language "C++"
		location ( solution_dir .. action )
		includedirs { external_dir, bounce_inc_dir, examples_inc_dir }
		vpaths { [""] = "testbed" }

		files 
		{ 
			examples_inc_dir .. "/testbed/framework/draw.h", 
			examples_inc_dir .. "/testbed/framework/profiler.h", 
			examples_inc_dir .. "/testbed/framework/recorder_profiler.h", 
			examples_inc_dir .. "/testbed/framework/json_profiler.h", 
			examples_inc_dir .. "/testbed/framework/testbed_listener.h", 
			
			examples_inc_dir .. "/testbed/framework/model.h", 
			examples_inc_dir .. "/testbed/framework/view.h", 
			examples_inc_dir .. "/testbed/framework/view_model.h", 
			
			examples_src_dir .. "/testbed/framework/test.h", 
			
			examples_src_dir .. "/testbed/framework/body_dragger.h", 
			examples_src_dir .. "/testbed/framework/cloth_dragger.h", 
			
			examples_src_dir .. "/testbed/framework/body_dragger.cpp", 
			examples_src_dir .. "/testbed/framework/cloth_dragger.cpp", 
			
			examples_inc_dir .. "/testbed/tests/**.h", 
			
			examples_src_dir .. "/testbed/framework/draw.cpp", 
			examples_src_dir .. "/testbed/framework/profiler.cpp", 
			examples_src_dir .. "/testbed/framework/recorder_profiler.cpp", 
			examples_src_dir .. "/testbed/framework/json_profiler.cpp", 
			
			examples_inc_dir .. "/testbed/framework/model.cpp", 
			examples_inc_dir .. "/testbed/framework/view.cpp", 
			examples_inc_dir .. "/testbed/framework/view_model.cpp", 
			
			examples_src_dir .. "/testbed/framework/test.cpp", 
			examples_src_dir .. "/testbed/framework/test_entries.cpp", 
			
			examples_src_dir .. "/testbed/framework/main.cpp" 
		}
		
		filter "options:gfxapi=opengl_2"
			files 
			{ 
				examples_src_dir .. "/testbed/framework/draw_gl2.h" 
			}			
			
		filter "options:gfxapi=opengl_4" 
			files 
			{ 
				examples_src_dir .. "/testbed/framework/draw_gl4.h" 
			}	
			
		filter "system:windows" 
			links { "opengl32", "winmm" }
			
		filter "system:linux" 
			links { "GL", "X11", "Xrandr", "Xinerama", "Xcursor", "pthread", "dl" }
		
		filter {}
		
		links { "glfw", "glad", "imgui", "bounce", "triangle" }
			
	project "hello_world"
		kind "ConsoleApp"
		language "C++"
		location ( solution_dir .. action )
		includedirs { bounce_inc_dir, examples_inc_dir }
		vpaths { ["Headers"] = "**.h", ["Sources"] = "**.cpp" }

		files 
		{ 
			examples_inc_dir .. "/hello_world/**.h", 
			examples_src_dir .. "/hello_world/**.cpp" 
		}

		links { "bounce" }

-- build
if os.istarget("windows") then
	
    newaction
    {
        trigger = "solution_vs2015",
        description = "Generate solution",
        execute = function ()
	    os.execute ( "premake5 clean" )
            os.execute ( "premake5 vs2015" )
        end
    }

	newaction
    {
        trigger = "solution_vs2017",
        description = "Generate solution",
        execute = function ()
	    os.execute ( "premake5 clean" )
            os.execute ( "premake5 vs2017" )
        end
    }

    newaction
    {
        trigger = "doc",
        description = "Generate documentation",
        execute = function ()
	    os.execute ( "doxygen doxyfile" )
	    os.execute ( "start doc\\api\\html\\index.html" )
        end
    }
end		

-- clean
newaction
{
    trigger = "clean",
    description = "Clean solution",
    execute = function ()
          os.rmdir( "doc" )
	  os.rmdir( solution_dir )
    end
}