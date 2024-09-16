**Coding Samples by Paul Cernea**

**Computational Geometry.**

* Navigate to the computational_geometry folder.
* If you are using Mac...

* Run `bash compile_and_run_geometry.sh` to run the computational geometry suite.  By default, 50 points are generated.
* Run `bash compile_and_run_geometry.sh <number-of-points>` to run the computational geometry suite with your desired number of points.  For instance `bash compile_and_run_geometry.sh 100` would run the program with 100 points.
* Click the window to generate a new point set.
* The convex hull is automatically generated.
* Press V to turn the Voronoi diagram on and off.
* Press D to turn Delaunay triangulation on and off.
* Press N to turn the Nearest-Neighbor graph on and off.
* Press T to turn naive triangulation on and off.
* See the screenshots seven_points.png, hundred_points_1.png, hundred_points_2.png for examples.
* Press **Q** or **ESC** to exit.

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

* This repo also contains a sample Django service written using Python, HTML, and JavaScript. It demonstrates finding the longest common subsequence of DNA strings. The source code along with a screenshot is contained in the `DjangoService` folder. To run this...
* Install Python and Django if you haven't already.
* Open a command prompt window and navigate to the DjangoService folder.
* Run the command `python manage.py makemigrations computationalBiology` to create a database.
* Run the command `python manage.py migrate computationalBiology`
* Run the command `python manage.py migrate`
* Run the command `python manage.py runserver` to run the service.
* Navigate to the locally hosted URL `http://localhost:8000/computationalBiology/`.
* The page should say "No computational biology algorithms available".
* Run the command `python manage.py createsuperuser`
* This will allow you to create an admin account. Do so, preferably with a simple password.
* Run the service again. Navigate to the locally hosted URL `http://localhost:8000/admin/` to log in.
* Now you can configure server-side content.
* Add an Algorithm. Choose 0 for Longest Common Subsequence.
* Add two Input Strings to this Algorithm containing default sequences like `GCATCATCATTAC` and `GACTACCATGCATACT` for the input fields. The captions can be arbitrary.
* Now navigate to the locally hosted URL `http://localhost:8000/computationalBiology/`.
* Clicking a link for the algorithm should take you to a page where you can compute the LCS.
* Running the algorithm for those two example strings should give the output `GCACATCATAC`.

* Besides the Django service, the `computational_biology` folder contains a CUDA computational biology implementation. See the CUDA and CMake instructions as listed above.
* Although the typical longest common substring algorithm does not leave a lot of room for parallelization, nevertheless parallel preprocessing can be performed. The longest common substring of N strings is at most as long as the LCS of any two of the strings. So by computing the smallest LCS of all pairs in parallel, we can obtain a bound to speed up the algorithm.
