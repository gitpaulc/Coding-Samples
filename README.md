**Coding Samples by Paul Cernea**

**Computational Geometry.**

* Navigate to the computational_geometry folder.
* If you are using Mac...

* Run `bash compile_and_run_geometry.sh` to run the computational geometry suite.  By default, 50 points are generated.
* Run `bash compile_and_run_geometry.sh <number-of-points>` to run the computational geometry suite with your desired number of points.  For instance `bash compile_and_run_geometry.sh 100` would run the program with 100 points.

* If you want to build the CUDA version on Windows, set the USE_CUDA CMake variable. Reconfigure and regenerate the project.
* Install CUDA if you haven't already.
* When you build, if it gives an error about not finding cl.exe, add this (or a similar) folder path as an environment variable: C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.41.34120\bin\Hostx64\x64\
* After adding the new PATH variable, close and open the command prompt to make sure it updates.
* Generally for Windows, follow the steps below.

* Otherwise, if you are using Windows...
* Use CMake to generate the ComputationalGeometry project/solution. If you do not have CMake installed, a project/solution is already generated in `Coding-Samples\computational_geometry\ComputationalGeometry`
* Use Release or Debug mode x64 (64-bit) if possible.
* If you do not have FreeGlut installed, install FreeGlut as follows:
* Download FreeGlut and build it if necessary.
* Copy the `freeglut` folder to `Coding-Samples\computational_geometry\ComputationalGeometry` or configure CMake to point to it.
* This `freeglut` folder should contain `include`, `lib`, and `bin` folders. The dependencies should now be in place.
* For OpenCV support, enable it from CMake. Otherwise uncomment the line `//#define USE_OPEN_CV` in includes.h.
* If using OpenCV, download OpenCV 4.9.0. Point CMake to the appropriate OpenCV folder, or create a folder called `opencv` in `Coding-Samples\computational_geometry\ComputationalGeometry` and copy the `build` folder in there.
* Build and run the executable from `Coding-Samples\computational_geometry\ComputationalGeometry\ComputationalGeometry.sln`
* By default 50 points are generated.
* If you pass arguments to the executable, the first argument is the custom number of points. For instance 100 for 100 points.

* Click the window to generate a new point set.
* The convex hull is automatically generated.
* Press V to turn the Voronoi diagram on and off.
* Press D to turn Delaunay triangulation on and off.
* Press N to turn the Nearest-Neighbor graph on and off.
* Press T to turn naive triangulation on and off.
* See the screenshots seven_points.png, hundred_points_1.png, hundred_points_2.png for examples.
* Press **Q** or **ESC** to exit.

**Computational Biology.**

* This repo also contains a sample Django service written using Python, HTML, and JavaScript. It demonstrates finding the longest common subsequence of DNA strings. The source code along with a screenshot is contained in the `DjangoService` folder.
