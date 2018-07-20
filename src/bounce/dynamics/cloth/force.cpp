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

#include <bounce/dynamics/cloth/force.h>
#include <bounce/dynamics/cloth/spring_force.h>
#include <bounce/dynamics/cloth/bend_force.h>

b3Force* b3Force::Create(const b3ForceDef* def)
{
	b3Force* force = NULL;
	switch (def->type)
	{
	case e_springForce:
	{
		void* block = b3Alloc(sizeof(b3SpringForce));
		force = new (block) b3SpringForce((b3SpringForceDef*)def);
		break;
	}
	case e_bendForce:
	{
		void* block = b3Alloc(sizeof(b3BendForce));
		force = new (block) b3BendForce((b3BendForceDef*)def);
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	}
	return force;
}

void b3Force::Destroy(b3Force* force)
{
	B3_ASSERT(force);

	b3ForceType type = force->GetType();
	switch (type)
	{
	case e_springForce:
	{
		b3SpringForce* o = (b3SpringForce*)force;
		o->~b3SpringForce();
		b3Free(force);
		break;
	}
	case e_bendForce:
	{
		b3BendForce* o = (b3BendForce*)force;
		o->~b3BendForce();
		b3Free(force);
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	};
}