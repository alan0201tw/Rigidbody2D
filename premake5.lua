workspace "Rigidbody2D"
    architecture "x64"
    startproject "Rigidbody2D"

    configurations
    {
        -- only debug for this hobby project
        "Debug",
        "Release"
    }

-- variables
    -- cfg - configuration
outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"

project "Rigidbody2D"
    location "."
    kind "ConsoleApp"  
    language "C++"
    staticruntime "off"

    targetdir ("bin/" .. outputdir .. "/%{prj.name}")
    objdir ("bin-int/" .. outputdir .. "/%{prj.name}")
    
    files
    {
        "include/**.hpp",
        "src/**.cpp"
    }

    includedirs
    {
        "include",
        "vendor/include"
    }

    libdirs
    {
        "vendor/lib/x64"
    }

    links
    {
        "freeglut"
    }

    postbuildcommands
    {
        -- copy         dll                                  to executable directory
        ("{COPY}   \"vendor/dll/x64/freeglut.dll\"   \"bin/" .. outputdir .. "/%{prj.name}/\"")
    }

    -- everything under this filter only applies to windows
    filter "system:windows"
        cppdialect "C++17"
        systemversion "latest"

        defines
        {
            "PLATFORM_WINDOWS",
            "_USE_MATH_DEFINES"
        }

    filter  "configurations:Debug"
        symbols "On"
    
    filter { "configurations:Release" }
        optimize "On"