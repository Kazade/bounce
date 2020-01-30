/*
* Copyright (c) 2016-2019 Irlan Robson https://irlanrobson.github.io
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

#include <bounce/softbody/softbody_force_solver.h>
#include <bounce/softbody/softbody_mesh.h>
#include <bounce/softbody/softbody_node.h>
#include <bounce/softbody/softbody_element.h>
#include <bounce/softbody/softbody.h>
#include <bounce/sparse/sparse_mat33.h>
#include <bounce/sparse/sparse_mat33_view.h>

// This work is based on the paper "Interactive Virtual Materials" written by 
// Matthias Mueller Fischer
// The paper is available here:
// http://matthias-mueller-fischer.ch/publications/GI2004.pdf

// In order to support velocity constraints on node velocities 
// we solve Ax = b using a Modified Preconditioned Conjugate Gradient (MPCG) algorithm.

// Number of MPCG iterations, a value that is normally small when small time steps are taken.
u32 b3_softBodySolverIterations = 0;

// Enables the stiffness warping solver.
bool b3_enableStiffnessWarping = true;

b3SoftBodyForceSolver::b3SoftBodyForceSolver(const b3SoftBodyForceSolverDef& def)
{
	m_step = def.step;
	m_body = def.body;
	m_stack = &m_body->m_stackAllocator;
	m_mesh = m_body->m_mesh;
	m_nodes = m_body->m_nodes;
	m_elements = m_body->m_elements;
}

b3SoftBodyForceSolver::~b3SoftBodyForceSolver()
{

}

// Extract rotation from deformation
// https://animation.rwth-aachen.de/media/papers/2016-MIG-StableRotation.pdf
static b3Quat b3ExtractRotation(const b3Mat33& A, const b3Quat& q0, u32 maxIterations = 20)
{
	b3Quat q = q0;

	for (u32 iteration = 0; iteration < maxIterations; ++iteration)
	{
		b3Mat33 R = q.GetXYZAxes();

		scalar s = b3Abs(b3Dot(R.x, A.x) + b3Dot(R.y, A.y) + b3Dot(R.z, A.z));

		if (s == scalar(0))
		{
			break;
		}

		const scalar kTol = scalar(1.0e-9);

		scalar inv_s = scalar(1) / s + kTol;

		b3Vec3 v = b3Cross(R.x, A.x) + b3Cross(R.y, A.y) + b3Cross(R.z, A.z);

		b3Vec3 omega = inv_s * v;

		scalar w = b3Length(omega);

		if (w < kTol)
		{
			break;
		}

		b3Quat omega_q;
		omega_q.SetAxisAngle(omega / w, w);

		q = omega_q * q;
		q.Normalize();
	}

	return q;
}

// Solve A * x = b
static void b3SolveMPCG(b3DenseVec3& x,
	const b3SparseMat33View& A, const b3DenseVec3& b,
	const b3DenseVec3& z, const b3DiagMat33& S, u32 maxIterations = 20)
{
	B3_PROFILE("Soft Body Solve MPCG");

	// Jacobi preconditioner
	// P = diag(A) 
	b3DiagMat33 P(A.rowCount);
	b3DiagMat33 invP(A.rowCount);
	for (u32 i = 0; i < A.rowCount; ++i)
	{
		b3Mat33 a = A(i, i);

		// Sylvester Criterion to ensure PD-ness
		B3_ASSERT(b3Det(a.x, a.y, a.z) > scalar(0));

		B3_ASSERT(a.x.x > scalar(0));
		scalar xx = scalar(1) / a.x.x;

		B3_ASSERT(a.y.y > scalar(0));
		scalar yy = scalar(1) / a.y.y;

		B3_ASSERT(a.z.z > scalar(0));
		scalar zz = scalar(1) / a.z.z;

		P[i] = b3Diagonal(a.x.x, a.y.y, a.z.z);
		invP[i] = b3Diagonal(xx, yy, zz);
	}

	x = z;

	scalar delta_0 = b3Dot(S * b, P * (S * b));

	b3DenseVec3 r = S * (b - A * x);
	b3DenseVec3 c = S * (invP * r);

	scalar delta_new = b3Dot(r, c);

	u32 iteration = 0;
	for (;;)
	{
		if (iteration == maxIterations)
		{
			break;
		}

		if (delta_new <= B3_EPSILON * B3_EPSILON * delta_0)
		{
			break;
		}

		b3DenseVec3 q = S * (A * c);

		scalar alpha = delta_new / b3Dot(c, q);

		x = x + alpha * c;
		r = r - alpha * q;

		b3DenseVec3 s = invP * r;

		scalar delta_old = delta_new;

		delta_new = b3Dot(r, s);

		scalar beta = delta_new / delta_old;

		c = S * (s + beta * c);

		++iteration;
	}

	b3_softBodySolverIterations = iteration;
}

void b3SoftBodyForceSolver::Solve(const b3Vec3& gravity)
{
	scalar h = m_step.dt;
	scalar inv_h = m_step.inv_dt;

	scalar alpha = m_body->m_massDamping;
	scalar beta = m_body->m_stiffnessDamping;

	b3DiagMat33 M(m_mesh->vertexCount);
	b3SparseMat33Pattern& KP = *m_body->m_KP;
	b3DenseVec3 x(m_mesh->vertexCount);
	b3DenseVec3 p(m_mesh->vertexCount);
	b3DenseVec3 v(m_mesh->vertexCount);
	b3DenseVec3 y(m_mesh->vertexCount);
	b3DenseVec3 f_elastic(m_mesh->vertexCount);
	b3DenseVec3 f_plastic(m_mesh->vertexCount);
	b3DenseVec3 fe(m_mesh->vertexCount);
	b3DenseVec3 z(m_mesh->vertexCount);
	b3DiagMat33 S(m_mesh->vertexCount);
	
	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		b3SoftBodyNode* n = m_nodes + i;
		B3_ASSERT(n->m_mass > scalar(0));
		M[i] = b3Diagonal(n->m_mass);
		x[i] = m_mesh->vertices[i];
		p[i] = n->m_position;
		y[i] = n->m_translation;
		v[i] = n->m_velocity;
		fe[i] = n->m_force;
		z[i] = n->m_velocity;

		// Apply gravity
		if (n->m_type == e_dynamicSoftBodyNode)
		{
			fe[i] += n->m_mass * gravity;
			S[i].SetIdentity();
		}
		else
		{
			S[i].SetZero();
		}
	}

	// Element assembly
	f_elastic.SetZero();
	f_plastic.SetZero();

	KP.SetZero();

	for (u32 ei = 0; ei < m_mesh->tetrahedronCount; ++ei)
	{
		b3SoftBodyMeshTetrahedron* mt = m_mesh->tetrahedrons + ei;
		b3SoftBodyElement* e = m_elements + ei;

		b3Mat33** Kp = e->m_Kp;
		b3Mat33* Ke = e->m_K;

		scalar* Be = e->m_B;
		scalar* Pe = e->m_P;
		scalar* epsilon_plastic = e->m_epsilon_plastic;

		u32 v1 = mt->v1;
		u32 v2 = mt->v2;
		u32 v3 = mt->v3;
		u32 v4 = mt->v4;

		b3Vec3 p1 = p[v1];
		b3Vec3 p2 = p[v2];
		b3Vec3 p3 = p[v3];
		b3Vec3 p4 = p[v4];

		b3Mat33 R;
		if (b3_enableStiffnessWarping)
		{
			b3Vec3 e1 = p2 - p1;
			b3Vec3 e2 = p3 - p1;
			b3Vec3 e3 = p4 - p1;

			b3Mat33 E(e1, e2, e3);

			b3Mat33 A = E * e->m_invE;

			b3Quat q = b3ExtractRotation(A, e->m_q);
			
			e->m_q = q;

			R = q.GetXYZAxes();
		}
		else
		{
			R.SetIdentity();
		}

		b3Mat33 RT = b3Transpose(R);

		for (u32 i = 0; i < 4; ++i)
		{
			for (u32 j = 0; j < 4; ++j)
			{
				*Kp[i + 4 * j] += R * Ke[i + 4 * j] * RT;
			}
		}

		// Elasticity
		b3Vec3 x1 = x[v1];
		b3Vec3 x2 = x[v2];
		b3Vec3 x3 = x[v3];
		b3Vec3 x4 = x[v4];

		// Displacements in unrotated frame
		b3Vec3 us[4];
		us[0] = RT * p1 - x1;
		us[1] = RT * p2 - x2;
		us[2] = RT * p3 - x3;
		us[3] = RT * p4 - x4;

		// Forces in unrotated frame
		b3Vec3 fs[4];
		for (u32 i = 0; i < 4; ++i)
		{
			fs[i].SetZero();
			for (u32 j = 0; j < 4; ++j)
			{
				fs[i] += Ke[i + 4 * j] * us[j];
			}
		}
		
		// Rotate the forces to deformed frame
		fs[0] = R * fs[0];
		fs[1] = R * fs[1];
		fs[2] = R * fs[2];
		fs[3] = R * fs[3];

		// Negate f
		f_elastic[v1] -= fs[0];
		f_elastic[v2] -= fs[1];
		f_elastic[v3] -= fs[2];
		f_elastic[v4] -= fs[3];

		// Plasticity

		// 6 x 1
		scalar epsilon_total[6];
		b3Mul(epsilon_total, Be, 6, 12, &us[0].x, 12, 1);

		// 6 x 1
		scalar epsilon_elastic[6];
		for (u32 i = 0; i < 6; ++i)
		{
			epsilon_elastic[i] = epsilon_total[i] - epsilon_plastic[i];
		}

		scalar len_epsilon_elastic = b3Length(epsilon_elastic, 6);
		if (len_epsilon_elastic > e->m_c_yield)
		{
			scalar amount = h * b3Min(e->m_c_creep, inv_h);
			for (u32 i = 0; i < 6; ++i)
			{
				epsilon_plastic[i] += amount * epsilon_elastic[i];
			}
		}

		scalar len_epsilon_plastic = b3Length(epsilon_plastic, 6);
		if (len_epsilon_plastic > e->m_c_max)
		{
			scalar scale = e->m_c_max / len_epsilon_plastic;
			for (u32 i = 0; i < 6; ++i)
			{
				epsilon_plastic[i] *= scale;
			}
		}

		b3Vec3 fs_plastic[4];
		b3Mul(&fs_plastic[0].x, Pe, 12, 6, epsilon_plastic, 6, 1);
		
		// Rotate the forces to deformed frame
		fs_plastic[0] = R * fs_plastic[0];
		fs_plastic[1] = R * fs_plastic[1];
		fs_plastic[2] = R * fs_plastic[2];
		fs_plastic[3] = R * fs_plastic[3];

		f_plastic[v1] += fs_plastic[0];
		f_plastic[v2] += fs_plastic[1];
		f_plastic[v3] += fs_plastic[2];
		f_plastic[v4] += fs_plastic[3];
	}

	b3SparseMat33 K(KP);

	// Rayleigh damping matrix
	b3SparseMat33 C = alpha * M + beta * K;

	// ODE:
	// M * a2 + C * v2 + K * (x2 - u) = f
	// where
	// x2 = x1 + h * v2
	// v2 = v1 + h * a2
	// a2 = (v2 - v1) / h
	// We identify the force due to the translation by rewriting the ODE:
	// M * a2 + C * v2 + K * ((x2 + y) - u)) = f
	// M * a2 + C * v2 + K * ((x2 - u) + y) = f
	// M * a2 + C * v2 + K * (x2 - u) + K * y = f
	// Solve for v2:
	// (M + h * C + h * h * K) * v2 = M * v1 + h * (fe - K * (x1 - u) - K * y)
	b3DenseVec3 f_translation = -(K * y);

	b3SparseMat33 A = M + h * C + h * h * K;

	b3SparseMat33View AV(A);

	b3DenseVec3 b = M * v + h * (fe + f_elastic + f_plastic + f_translation);

	b3DenseVec3 sx(m_mesh->vertexCount);
	b3SolveMPCG(sx, AV, b, z, S);

	// Copy velocity back to the nodes
	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		m_nodes[i].m_velocity = sx[i];
		m_nodes[i].m_position += y[i];
	}
}