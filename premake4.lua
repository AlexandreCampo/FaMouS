local action = _ACTION or ""

solution "famous"
   configurations { "debug", "release", "profile" }
 
   project "famous"
      kind "StaticLib"
      language "C++"
      files { "core/**.h", "core/**.cpp" }
 
      includedirs { "core/" }
      includedirs { "core/objects/" }
      includedirs { "core/services/" }
      includedirs { "core/devices/" }

      includedirs { "/usr/include/" }
      includedirs { "/usr/local/include/bullet" }
      includedirs { "/usr/local/include" }

      libdirs { os.findlib("glut"), os.findlib("GL"), os.findlib("GLU"), os.findlib("gsl"), os.findlib("BulletDynamics")}

      links { "gsl", "glut", "GL", "GLU", "BulletCollision", "BulletDynamics", "LinearMath", "gslcblas"}


      configuration "release"
         defines { "NDEBUG" }
         flags { "OptimizeSpeed", "EnableSSE", "EnableSSE2", "FloatFast", "NoExceptions", "NoFramePointer"}    
         postbuildcommands ( "find ./core/ -name *.h -exec cp -u {} include/ \\;" )
         postbuildcommands ( "mv *.a lib/" )

      configuration "debug"
         defines { "DEBUG" }
         flags { "Symbols" }
         postbuildcommands ( "find ./core/ -name *.h -exec cp -u {} include/ \\;" )
         postbuildcommands ( "mv *.a lib/" )

      configuration "profile"
         defines { "PROFILE" }
         flags { "Symbols" }
	 buildoptions { "-pg" }	
         postbuildcommands ( "find ./core/ -name *.h -exec cp -u {} include/ \\;" )
         postbuildcommands ( "mv *.a lib/" )


if action == "clean" then
  os.execute ("rm -f ./include/*.h")
  os.execute ("rm -f ./lib/*.a")
end
