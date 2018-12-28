# Visual Odometry
===
This code implements a very simple monocular visual odometry.


#### System Requirements ####

The reference code is tested and compiled on <b>windows 10, Ubuntu 16.04, macOS 10.14.</b> and requires:

- C++ 11 (Visual C++ Only 2017 has been tested, GCC)
- CMake (2.4 or higher)
- OpenCV (3.1 or higher - The lower version was not tested.) 

#### Build ####

Create a seperate build directory and generate a `Makefile`.
Running `make` on the generated `Makefile` builds the applicaton and adds it to the `bin` directory.

```bash
$ mkdir build
$ cd build
$ cmake ..
$ make
$ ./visual_odometry
```

windows 10 
1. Cmake is still available, you can use cmake to generate a visual studio project.
2. You can also enter the VC2017 directory and double-click visual_odometry.sln. Note that you may need to configure opencv yourself. My opencv is installed in d:\opencv3.4\opencv\build.  opencv version is opencv3.4.4

