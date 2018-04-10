/*
* Copyright (c) 2016-2016 Irlan Robson http://www.irlan.net
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include <bounce/collision/shapes/sphere.h>
#include <bounce/collision/shapes/capsule.h>
#include <bounce/collision/shapes/box_hull.h>

const b3Sphere b3Sphere_identity(b3Vec3_zero, 1.0f); 

const b3Capsule b3Capsule_identity(b3Vec3(0.0f, -0.5f, 0.0f), b3Vec3(0.0f, 0.5f, 0.0f), 1.0f);

const b3BoxHull b3BoxHull_identity(1.0f, 1.0f, 1.0f);