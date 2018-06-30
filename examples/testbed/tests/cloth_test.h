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

#ifndef CLOTH_TESH_H
#define CLOTH_TESH_H

class ClothTest : public Test
{
public:
	ClothTest()
	{
		m_world.SetGravity(b3Vec3(0.0f, -10.0f, 0.0f));
		m_cloth = nullptr;
	}

	void Step()
	{
		Test::Step();

		m_cloth->Apply();

		m_cloth->Draw();

		extern u32 b3_clothSolverIterations;
		g_draw->DrawString(b3Color_white, "Iterations = %u", b3_clothSolverIterations);
		
		float32 E = m_cloth->GetEnergy();
		g_draw->DrawString(b3Color_white, "E = %f", E);
	}
	
	b3Cloth* m_cloth;
};

#endif