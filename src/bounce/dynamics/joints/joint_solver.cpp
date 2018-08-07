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

#include <bounce/dynamics/joints/joint_solver.h>
#include <bounce/dynamics/joints/joint.h>

b3JointSolver::b3JointSolver(const b3JointSolverDef* def) 
{
	m_count = def->count;
	m_joints = def->joints;
	m_solverData.dt = def->dt;
	m_solverData.invdt = def->dt > 0.0f ? 1.0f / def->dt : 0.0f;
	m_solverData.positions = def->positions;
	m_solverData.velocities = def->velocities;
	m_solverData.invInertias = def->invInertias;
}

void b3JointSolver::InitializeConstraints() 
{
	for (u32 i = 0; i < m_count; ++i) 
	{
		b3Joint* j = m_joints[i];
		j->InitializeConstraints(&m_solverData);
	}
}

void b3JointSolver::WarmStart() 
{
	for (u32 i = 0; i < m_count; ++i) 
	{
		b3Joint* j = m_joints[i];
		j->WarmStart(&m_solverData);
	}
}

void b3JointSolver::SolveVelocityConstraints() 
{
	for (u32 i = 0; i < m_count; ++i) 
	{
		b3Joint* j = m_joints[i];
		j->SolveVelocityConstraints(&m_solverData);
	}
}

bool b3JointSolver::SolvePositionConstraints() 
{
	bool jointsSolved = true;
	for (u32 i = 0; i < m_count; ++i) 
	{
		b3Joint* j = m_joints[i];
		bool jointSolved = j->SolvePositionConstraints(&m_solverData);
		jointsSolved = jointsSolved && jointSolved;
	}
	return jointsSolved;
}
