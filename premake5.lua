-- Irlan Robson
-- Bounce premake script
-- http://industriousone.com/premake

-- paths as variables in the case files are moved
solution_name = "bounce"
working_dir = "."
solution_dir = "build/"
external_dir = "external/"
inc_dir = "include/"
src_dir = "src/"
obj_dir = "/obj/"
bin_dir = "/bin/"

action = _ACTION

-- premake main
solution (solution_name)
	location ( solution_dir .. "/" .. action )
	configurations { "debug", "release" }
	platforms { "x32", "x64" }
	
	filter { "platforms:x32" }
    system "windows"
    architecture "x32"

	filter { "platforms:x64" }
    system "windows"
    architecture "x64"
	
	-- note the use of "!" before objdir to force the specified path
	configuration "debug"
		targetdir ( solution_dir .. action .. bin_dir .. "%{cfg.platform}/%{cfg.buildcfg}/%{prj.name}" )
		objdir ( "!" .. solution_dir .. action .. obj_dir .. "%{cfg.platform}/%{cfg.buildcfg}/%{prj.name}" )
		defines { "_DEBUG" }
		symbols "On"
		rtti "Off"
		flags { "FloatFast" }
			
    configuration "release"
		targetdir ( solution_dir .. action .. bin_dir .. "%{cfg.platform}/%{cfg.buildcfg}/%{prj.name}" )
		objdir ( "!" .. solution_dir .. action .. obj_dir .. "%{cfg.platform}/%{cfg.buildcfg}/%{prj.name}" )
		defines { "NDEBUG" }
		optimize "On"
		rtti "Off"
		flags { "FloatFast" }
		
	configuration "windows"
		defines { "WIN32", "_WINDOWS" }
		
	project "bounce"
		kind "StaticLib"
		language "C++"
		location ( solution_dir .. action )
		files 
		{ 
			inc_dir .. "/bounce/**.h", 
			inc_dir .. "/bounce/**.inl", 
			src_dir .. "/bounce/**.cpp" 
		}
		includedirs { inc_dir }
		vpaths { [""] = "bounce" }
		
	project "glad"
		kind "StaticLib"
		language "C"
		location ( solution_dir .. action )
		files 
		{ 
			external_dir .. "/glad/**.h", 
			external_dir .. "/glad/**.c" 
		}
		includedirs { external_dir }
		defines { "_CRT_SECURE_NO_WARNINGS" } 
		vpaths { ["Headers"] = "**.h", ["Sources"] = "**.c" }		
		 
	project "glfw"
		kind "StaticLib"
		language "C"
		location ( solution_dir .. action )
		
		-- windows headers and sources
		configuration { "windows" }
			files 
			{	 
				external_dir .. "/glfw/egl_context.h",
				external_dir .. "/glfw/glfw3.h",
				external_dir .. "/glfw/glfw3native.h",
				external_dir .. "/glfw/glfw_config.h",
				external_dir .. "/glfw/wgl_context.h",
				external_dir .. "/glfw/win32_joystick.h",
				external_dir .. "/glfw/win32_platform.h",

				external_dir .. "/glfw/context.c", 		
				external_dir .. "/glfw/egl_context.c",
				external_dir .. "/glfw/init.c",
				external_dir .. "/glfw/input.c",
				external_dir .. "/glfw/monitor.c",
				external_dir .. "/glfw/vulkan.c",
				external_dir .. "/glfw/wgl_context.c",
				external_dir .. "/glfw/win32_init.c",
				external_dir .. "/glfw/win32_joystick.c",
				external_dir .. "/glfw/win32_monitor.c",
				external_dir .. "/glfw/win32_time.c",
				external_dir .. "/glfw/win32_tls.c",
				external_dir .. "/glfw/win32_window.c",
				external_dir .. "/glfw/window.c"
			}
		includedirs { external_dir .. "/glfw" }
		defines { "_GLFW_USE_CONFIG_H", "_CRT_SECURE_NO_WARNINGS" } 
		vpaths { ["Headers"] = "**.h", ["Sources"] = "**.c" }		
		 
	project "imgui"
		kind "StaticLib"
		language "C++"
		location ( solution_dir .. action )
		files 
		{ 
			external_dir .. "/imgui/**.h", 
			external_dir .. "/imgui/**.cpp" 
		}
		includedirs { external_dir }
		defines { "_CRT_SECURE_NO_WARNINGS" } 
		vpaths { ["Headers"] = "**.h", ["Sources"] = "**.cpp" }		

	project "testbed"
		kind "ConsoleApp"
		language "C++"
		location ( solution_dir .. action )
		files 
		{ 
			inc_dir .. "/testbed/**.h", 
			src_dir .. "/testbed/**.cpp" 
		}
		includedirs { external_dir, inc_dir }
		
		-- dependencies
		links { "bounce", "glad", "glfw", "imgui" }
		
		-- windows dependencies
		configuration { "windows" }
			links { "glu32", "opengl32", "winmm" }
		vpaths { ["Headers"] = "**.h", ["Sources"] = "**.cpp" }

-- build
if os.is "windows" then
	
    newaction
	{
        trigger = "solution",
        description = "Make and open solution",
        execute = function ()
		    os.execute ( "premake5 clean" )
            os.execute ( "premake5 vs2015" )
            os.execute ( "start " .. solution_dir .. "vs2015/bounce.sln" )
        end
    }

end		

-- clean
newaction
{
    trigger = "clean",
    description = "Clean solution",
    execute = function ()
          os.rmdir( solution_dir )
    end
}