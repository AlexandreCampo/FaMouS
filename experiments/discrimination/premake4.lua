

solution "discrimination"
   configurations { "debug", "release" , "profile" }

   --- location of the simulator 
   includedirs { "../../include/" }
   libdirs { "../../lib/" }

   --- other libs
   includedirs { "/usr/include/" }
   includedirs { "/usr/local/include/bullet" }
   includedirs { "/usr/local/include" }

   libdirs { os.findlib("glut"), os.findlib("GL"), os.findlib("GLU"), os.findlib("gsl"), os.findlib("BulletDynamics"), os.findlib("boost_program_options"), os.findlib("tinyxml")}
   links { "famous", "BulletDynamics", "BulletCollision", "LinearMath", "gsl", "gslcblas", "glut", "GL", "GLU", "boost_program_options", "tinyxml"}


   project "discrimination"
      kind "ConsoleApp"
      language "C++"
      files { "**.h", "**.cpp" }

      configuration "release"
         defines { "NDEBUG" }
         flags { "OptimizeSpeed", "EnableSSE", "EnableSSE2", "FloatFast", "NoFramePointer"}    

      configuration "debug"
         defines { "DEBUG" }
         flags { "Symbols" }
 
      configuration "profile"
         defines { "DEBUG" }
         flags { "Symbols" }
	 buildoptions { "-pg" }	
	 linkoptions { "-pg" }	
