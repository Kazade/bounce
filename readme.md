## Bounce

**Welcome! Bounce is a 3D physics engine for games.**

Feel free to ask questions, give feedback and suggestions using the issue tracker. 

## License

Bounce is released under the [zlib license](https://en.wikipedia.org/wiki/Zlib_License). Please recognize this software in the product documentation if possible.

## Building

Bounce uses [premake](https://premake.github.io/) for generating project files in a platform agnostic manner. [premake](https://premake.github.io/) is available at https://premake.github.io/.

* Put premake into bounce/.

### Visual Studio 2017

* Ensure you have installed the Visual Studio 2015 libraries.
* Say { premake5 vs2017 } on a command line. 
* Open build/vs2017/bounce.sln.
* Set testbed as the startup project.
* In the testbed debugging properties, set the Working Directory to ..\..\examples\testbed.
* Press F5 to run.

### Linux

* On a clean Ubuntu 16.04 install these packages first:

* mesa-common-dev

* libgl1-mesa-dev

* libglu1-mesa-dev 

#### x32

* Say { premake5 gmake } on a terminal.
* From build/gmake say { make config="debug_x32" }.
* Set the testbed directory as the working directory.
* Open testbed from /bin/x32/testbed/.

#### x64

* Say { premake5 gmake } on a terminal.
* From build/gmake say { make config="debug_x64" }.
* Set the testbed directory as the working directory.
* Open testbed from /bin/x64/testbed/.

### Mac

I don't run Mac currently and therefore can't test the build system in this platform.

## Documentation

* [Quickstart Guide](https://github.com/irlanrobson/bounce/blob/master/doc/quickstart_guide.docx)

### Doxygen

* Grab [Doxygen](http://www.doxygen.org) from http://www.doxygen.org.
* Say the following on the command line: doxygen doxyfile.
* Open doc/api/html/index.html.

User manual is a work in progress. Meanwhile, code comments and examples are the best way to learn how to use 
Bounce. For examples, you can use the project testbed. testbed is a collection of visual tests and examples that can support the development of the library. As you would imagine, this application is not part of the library.

## Contributing

Please do not open pull requests with bugfixes or new features that require large changes. Open an issue first for discussion.

## Dependencies

* [Triangle](http://www.cs.cmu.edu/~quake/triangle.html)

Below are the external dependencies for testbed. If you don't care about testbed, then you don't need these dependencies. 

* [GLFW](https://www.glfw.org/)
* [GLAD](https://glad.dav1d.de/)
* [imgui](https://github.com/ocornut/imgui)
* [RapidJSON](http://rapidjson.org/index.html)

## Features

### Common

* Efficient data structures with no use of STL
* Stack and small block allocators
* Built-in math library
* Tunable settings used across the entire library

### Quickhull

* Robust 3D convex hull creation and simplification

### Collision

* Dynamic tree broadphase
* Static tree "midphase"
* SAT
* GJK
* Spheres, capsules, convex hulls, triangle meshes
* Optimized pair management

### Dynamics

* Rigid bodies
* Cloth
* Soft body
* Contact, friction, restitution
* Mouse, spring, sphere, cone, revolute joint types
* Quaternion constraints
* Joint motors, limits
* Constraint graphs
* Simulation islands and sleep management
* Linear time solver
* Stable shape stacking
* One-shot contact manifolds
* Contact clustering, reduction, and persistence
* Contact callbacks: begin, pre-solve, post-solve
* Ray-casting and volume queries

### Testbed
	
* OpenGL with GLFW and GLAD
* UI by imgui
* Mouse picking
* premake build system

### Documentation

* Doxygen API documentation</li>
