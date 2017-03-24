### About

Bounce is a 3D physics engine for games.

To get started with Bounce see readme.txt.

Here is how to control the Testbed. The Testbed is a collection of non-unit tests and examples that helps the author debug features and is not part of the library.

Camera (as in Maya):

* Rotate the scene holding LSHIFT + LMB
* Translate the scene holding LSHIFT + RMB
* Zoom in/out the scene using LSHIFT + Mouse Wheel

Bounce is released under the <b>zlib</b> license.

### Features

#### Common

* Efficient data structures with no use of STL
* Stack and small block allocators
* Built-in math library
* Tunable settings used across the entire library

#### Collision

* Dynamic tree broadphase
* Static tree "midphase"
* SAT
* GJK
* Spheres, capsules, hulls, triangle meshes
* Optimized pair management

#### Dynamics

* Contact, friction, restitution
* Mouse, spring, sphere, cone, revolute joint types
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