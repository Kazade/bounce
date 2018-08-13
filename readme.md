### About

Bounce is a 3D physics engine.

To get started with Bounce see readme.txt.

Inside the examples folder there is the source code for an application called Testbed. The Testbed is a collection of visual tests and examples that can support the development of the library. As you would imagine, this application is not part of the library. However, it's still recommended to read the code to become confortable with using the library. It might take some time to document Bounce properly.

Bounce is released under the <b>zlib</b> license.

### Features

#### Common

* Efficient data structures with no use of STL
* Stack and small block allocators
* Built-in math library
* Tunable settings used across the entire library

#### Quickhull

* Robust 3D convex hull creation and simplification

#### Collision

* Dynamic tree broadphase
* Static tree "midphase"
* SAT
* GJK
* Spheres, capsules, convex hulls, triangle meshes
* Optimized pair management

#### Dynamics

* Rigid bodies
* Cloth
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

#### Testbed
	
* OpenGL with GLFW and GLAD
* UI by imgui
* Mouse picking
* premake build system

#### Documentation

* Doxygen API documentation</li>

#### Screenshots

![screenshot 1](/screenshots/a.png?raw=true)
