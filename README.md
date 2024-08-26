**Coding Samples by Paul Cernea**

**Computational Geometry.**

* Navigate to the computational_geometry folder.
* If you are using Mac...

* Run `bash compile_and_run_geometry.sh` to run the computational geometry suite.  By default, 50 points are generated.
* Run `bash compile_and_run_geometry.sh <number-of-points>` to run the computational geometry suite with your desired number of points.  For instance `bash compile_and_run_geometry.sh 100` would run the program with 100 points.

* If you are using Windows...
* If you do not have Glut installed, install Glut as follows:
* Download the Glut zip file, for instance from https://www.opengl.org/resources/libraries/glut/glutdlls37beta.zip.
* For simplicity, after unzipping the folder, rename it to `glut`. The folder contains `glut.h`,`glut.lib`, `glut32.lib`, `glut.dll`, and `glut32.dll`.
* Navigate to `C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\VS\include`. If a folder called `GL` does not exist there, create it, and copy `glut.h` there.
* Copy `glut32.lib` to `C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\VS\lib\x86`.
* Copy `glut.lib` to `C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\VS\lib\x64`.
* Copy `glut32.dll` to `C:\Windows\System32`.
* Copy `glut.dll` to `C:\Windows\SysWOW64`.
* Now that Glut is installed, copy the downloaded glut folder to `Coding-Samples\computational_geometry\ComputationalGeometry\glut`.
* This folder should contain `glut32.dll` and `glut.dll`.
* Having this folder in place will allow the post-build script to copy the DLLs in the build folder.
* For now, make sure to build in 32-bit mode.
* Build and run the executable from `Coding-Samples\computational_geometry\ComputationalGeometry\ComputationalGeometry.sln`
* By default 50 points are generated.
* If you pass arguments to the executable, the first argument is the custom number of points. For instance 100 for 100 points.

* Click the window to generate a new point set.
* The convex hull is automatically generated.
* Press D to turn Delaunay triangulation on and off.
* Press T to turn naive triangulation on and off.
* See the screenshots seven_points.png, hundred_points_1.png, hundred_points_2.png for examples.
* Press **Q** or **ESC** to exit.
