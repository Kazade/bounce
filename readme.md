<h3>About</h3>

Bounce is a 3D physics engine for games and interactive applications.

To get started with Bounce see readme.txt.

Here is how to control the Testbed. The Testbed is a collection of non-unit tests and examples that helps the author debug features and is not part of the library.

Camera (as in Maya):

<ul>
	<li>Rotate the scene holding LSHIFT + LMB</li>
	<li>Translate the scene holding LSHIFT + RMB</li>
	<li>Zoom in/out the scene using LSHIFT + Mouse Wheel</li>
</ul>

Bounce is released under the <b>zlib</b> license.

<h3>Features</h3>

<ul>
<h4>Common</h4>

	<li>Efficient data structures with no use of STL</li>
	<li>Stack and small block allocators</li>
	<li>Built-in math library</li>
	<li>Tunable settings used across the entire library</li>

<h4>Collision</h4>

	<li>Dynamic tree broadphase</li>
	<li>Static tree "midphase"</li>
	<li>SAT</li>
	<li>GJK</li>
	<li>Spheres, capsules, hulls, triangle meshes</li>
	<li>Optimized pair management</li>

<h4>Dynamics</h4>

	<li>Contact, friction, restitution</li>
	<li>Mouse, spring, sphere, cone, revolute joint types</li>
	<li>Joint motors, limits</li>
	<li>Constraint graphs</li>
	<li>Simulation islands and sleep management</li>
	<li>Linear time solver</li>
	<li>Stable shape stacking</li>
	<li>One-shot contact manifolds</li>
	<li>Contact clustering, reduction, and persistence</li>
	<li>Contact callbacks: begin, pre-solve, post-solve</li>
	<li>Ray-casting and volume queries</li>

<h4>Testbed</h4>
	
	<li>OpenGL with GLFW and GLAD</li>
	<li>UI by imgui</li>
	<li>Mouse picking</li>
	<li>premake build system</li>

<h4>Documentation</h4>

	<li>Doxygen API documentation</li>
</ul>
