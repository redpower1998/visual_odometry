# Visual Odometry
===
This code implements a very simple monocular visual odometry.

Video: [https://youtu.be/RfI6pX9trac](https://youtu.be/RfI6pX9trac)<br/>

## Demo Video

[![Demo video](2011_09_30_clip1.gif)](https://youtu.be/RfI6pX9trac)

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

```Windows 
1. Cmake is still available, you can use cmake to generate a visual studio project.
2. You can also enter the VC2017 directory and double-click visual_odometry.sln. Note that you may need to configure opencv yourself. My opencv is installed in d:\opencv3.4\opencv\build.  opencv version is opencv3.4.4

Note that I only tested VS2017 and I believe VS2015 will work correctly, but I have not tested it.
```


#### Known issues ####

You need to modify the camera parameters in your code yourself. This is very important. These intrinsic parameters can be obtained by calibration. If you use public test cases, these test cases usually contain intrinsic parameters.

Below are the intrinsic parameters obtained from the test case.
```Samplexiamia
<?xml version="1.0"?>
<opencv_storage>
<FocalLength>780.1981</FocalLength>
<CenterX>660.1406</CenterX>
<CenterY>272.1004</CenterY>
</opencv_storage>
```
These parameters correspond to the source code.
```
double focal = 780.1981;
cv::Point2d pp(660.1406, 272.1004);   //intrinsic parameters
```

#### License ####
BSD 2-Clause License. See LICENSE.txt for further details.

