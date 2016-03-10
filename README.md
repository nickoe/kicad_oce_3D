# kicad_oce_3D

This project is an out-of-tree 3D visualization plugin
for the KiCad 3d_initial_merge branch. The plugin uses
OpenCASCADE Community Edition (OCE) to provide the ability
to view STEP and IGES solid models and assemblies.

In addition to the existing KiCad dependencies you need
to install OpenCascade Community Edition:

https://github.com/tpaviot/oce

At the moment the build has only been tested on Linux
with OCE as supplied by the package manager. It is
assumed that the OCE package provides suitable
FindOCE scripts for CMake. For OSX and MSWin developers,
please provide feedback on installing OCE on those
systems.

Building on Linux:
1. change into the kicad_oce_3D directory
2. create a build directory: mkdir build && cd build
3. configure and build: cmake .. && make

Note on building OCE on MSWin:
(as provided by Jean-Pierre Charras, creator of KiCad)

The top level OCE CMakeLists.txt needs to be patched
to build on W7-32 bit using MSys2:

```
iff --git a/CMakeLists.txt b/CMakeLists.txt
index 29891da..2b18115 100644
--- a/CMakeLists.txt
+++ b/CMakeLists.txt
@@ -125,6 +125,18 @@ if(WIN32)
        mark_as_advanced(OCE_AUTOINSTALL_DEPENDENT_LIBS)
 endif()

+
+if( MSYS )
+    # JPC: use a response file for include and link, because the environment buffer is too small
+    # when there are a lot of files to link or include
+    set( CMAKE_C_USE_RESPONSE_FILE_FOR_INCLUDES 1 )
+    set( CMAKE_CXX_USE_RESPONSE_FILE_FOR_INCLUDES 1 )
+    set( CMAKE_C_USE_RESPONSE_FILE_FOR_OBJECTS 1 )
+    set( CMAKE_CXX_USE_RESPONSE_FILE_FOR_OBJECTS 1 )
+    #set( CMAKE_CXX_RESPONSE_FILE_LINK_FLAG "@" )
+endif()
+
+
 if ((MSVC AND NOT NMAKE) OR CMAKE_COMPILER_IS_GNUCXX)
        set(OCE_COMPILER_SUPPORTS_PCH TRUE)
        if(MSVC)
```
