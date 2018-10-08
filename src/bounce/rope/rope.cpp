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

#include <bounce/rope/rope.h>
#include <bounce/rope/spatial.h>
#include <bounce/common/draw.h>

struct b3RopeBody
{
	b3RopeBody() { }

	// J * v
	b3MotionVec v_J() const
	{
		return m_S[0] * m_v[0] + m_S[1] * m_v[1] + m_S[2] * m_v[2];
	}

	// 
	b3Transform X_J() const
	{
		// Rigid Body Dynamics Algorithms p. 86
		// E = mat33(inv(p))
		b3Quat E = b3Conjugate(m_p);

		b3Transform X;
		X.rotation = b3QuatMat33(E);
		X.position.SetZero();
		return X;
	}

	// Shared

	// Body

	//
	float32 m_m, m_I;

	// Joint

	//
	b3MotionVec m_S[3];

	//
	b3Transform m_X_i_J;

	// 
	b3Transform m_X_J_j;

	//
	b3Quat m_p;

	//
	b3Vec3 m_v;

	// Temp

	//
	b3SpTransform m_X_i_j;

	//
	b3MotionVec m_sv;

	//
	b3MotionVec m_sc;

	//
	b3SpInertia m_I_A;

	//
	b3ForceVec m_F_A;

	//
	b3ForceVec m_U[3];

	//
	b3Mat33 m_invD;

	//
	b3Vec3 m_u;

	//
	b3Vec3 m_a;

	//
	b3MotionVec m_sa;

	//
	b3Transform m_invX;

	//
	b3Transform m_X;
};

b3Rope::b3Rope()
{
	m_gravity.SetZero();
	m_kd1 = 0.0f;
	m_kd2 = 0.0f;
	m_links = NULL;
	m_count = 0;
}

b3Rope::~b3Rope()
{
	b3Free(m_links);
}

void b3Rope::Initialize(const b3RopeDef& def)
{
	B3_ASSERT(def.count > 0);

	m_gravity = def.gravity;
	m_kd1 = def.linearDamping;
	m_kd2 = def.angularDamping;
	m_count = def.count;
	m_links = (b3RopeBody*)b3Alloc(m_count * sizeof(b3RopeBody));

	for (u32 i = 0; i < m_count; ++i)
	{
		b3RopeBody* b = m_links + i;
		
		float32 m = def.masses[i];
		
		// Simplify r = 1
		b->m_m = m;
		b->m_I = m * 0.4f;
	}

	m_v.SetZero();
	m_p = def.vertices[0];
	m_w.SetZero();
	m_q.SetIdentity();

	m_links->m_X.rotation = b3QuatMat33(m_q);
	m_links->m_X.position = m_p;

	for (u32 i = 1; i < m_count; ++i)
	{
		b3RopeBody* b = m_links + i;
		b3RopeBody* b0 = b - 1;
		b3Vec3 p = def.vertices[i];
		b3Vec3 p0 = def.vertices[i - 1];

		b->m_X.rotation.SetIdentity();
		b->m_X.position = p;

		// Set the joint anchor to the parent body position to simulate a rope.
		b3Transform X_J;
		X_J.rotation.SetIdentity();
		X_J.position = p0;

		b->m_X_i_J = b3MulT(X_J, b0->m_X);
		
		b->m_X_J_j = b3MulT(b->m_X, X_J);

		b3Vec3 d = -b->m_X_J_j.position;

		b3Vec3 w1(b3Vec3_x);
		b3Vec3 v1 = b3Cross(w1, d);
		
		b3Vec3 w2(b3Vec3_y);
		b3Vec3 v2 = b3Cross(w2, d);
		
		b3Vec3 w3(b3Vec3_z);
		b3Vec3 v3 = b3Cross(w3, d);
		
		b->m_S[0].w = w1;
		b->m_S[0].v = v1;

		b->m_S[1].w = w2;
		b->m_S[1].v = v2;

		b->m_S[2].w = w3;
		b->m_S[2].v = v3;

		b->m_p.SetIdentity();
		b->m_v.SetZero();
	}
}

void b3Rope::Step(float32 h)
{
	if (m_count == 0)
	{
		return;
	}

	// Propagate down.
	{
		b3RopeBody* b = m_links;
		b3Mat33 I = b3Diagonal(b->m_I);

		b->m_invX = b3Inverse(b->m_X);

		// Convert global velocity to local velocity.
		b->m_sv.w = b->m_invX.rotation * m_w;
		b->m_sv.v = b->m_invX.rotation * m_v;

		if (b->m_m == 0.0f)
		{
			b->m_I_A.SetZero();
			b->m_F_A.SetZero();
		}
		else
		{
			// Uniform inertia results in zero angular momentum.
			b3ForceVec Pdot;
			Pdot.n = b3Cross(b->m_sv.w, b->m_m * b->m_sv.v);
			Pdot.f.SetZero();

			// Convert global force to local force.
			b3ForceVec F;
			F.n = b->m_invX.rotation * m_gravity;
			F.f.SetZero();
			
			// Damping force
			b3ForceVec Fd;
			Fd.n = -m_kd1 * b->m_m * b->m_sv.v;
			Fd.f = -m_kd2 * b->m_I * b->m_sv.w;

			b->m_I_A.SetLocalInertia(b->m_m, I);
			b->m_F_A = Pdot - (F + Fd);
		}
	}

	for (u32 i = 1; i < m_count; ++i)
	{
		b3RopeBody* link = m_links + i;
		b3RopeBody* parent = link - 1;
		b3Mat33 I = b3Diagonal(link->m_I);

		b3Transform X_J = link->X_J();
		b3Transform X_i_j = link->m_X_J_j * X_J * link->m_X_i_J;

		link->m_invX = X_i_j * parent->m_invX;

		// Flip the translation because r should be the vector 
		// from the center of mass of the parent link to the 
		// center of mass of this link in this link's frame.
		link->m_X_i_j.E = X_i_j.rotation;
		link->m_X_i_j.r = -X_i_j.position;

		b3MotionVec joint_v = link->v_J();
		
		b3MotionVec parent_v = b3Mul(link->m_X_i_j, parent->m_sv);

		link->m_sv = parent_v + joint_v;
		
		// v x jv
		link->m_sc.w = b3Cross(link->m_sv.w, joint_v.w);
		link->m_sc.v = b3Cross(link->m_sv.v, joint_v.w) + b3Cross(link->m_sv.w, joint_v.v);

		// Uniform inertia results in zero angular momentum.
		b3ForceVec Pdot;
		Pdot.n = b3Cross(link->m_sv.w, link->m_m * link->m_sv.v);
		Pdot.f.SetZero();

		// Damping force
		b3ForceVec Fd;
		Fd.n = -m_kd1 * link->m_m * link->m_sv.v;
		Fd.f = -m_kd2 * link->m_I * link->m_sv.w;

		// Convert global force to local force.
		b3ForceVec F;
		F.n = link->m_invX.rotation * m_gravity;
		F.f.SetZero();
				
		link->m_I_A.SetLocalInertia(link->m_m, I);
		link->m_F_A = Pdot - (F + Fd);
	}
	
	// Propagate up bias forces and inertias.
	for (u32 j = m_count - 1; j >= 1; --j)
	{
		b3RopeBody* link = m_links + j;
		b3RopeBody* parent = link - 1;
		b3MotionVec* S = link->m_S;
		b3MotionVec& c = link->m_sc;

		b3SpInertia& I_A = link->m_I_A;
		b3ForceVec& F_A = link->m_F_A;

		b3ForceVec* U = link->m_U;
		b3Vec3& u = link->m_u;
				
		// U
		U[0] = I_A * S[0];
		U[1] = I_A * S[1];
		U[2] = I_A * S[2];

		// D = S^T * U
		b3Mat33 D;

		D.x.x = b3Dot(S[0], U[0]);
		D.x.y = b3Dot(S[1], U[0]);
		D.x.z = b3Dot(S[2], U[0]);

		D.y.x = D.x.y;
		D.y.y = b3Dot(S[1], U[1]);
		D.y.z = b3Dot(S[2], U[1]);

		D.z.x = D.x.z;
		D.z.y = D.y.z;
		D.z.z = b3Dot(S[2], U[2]);

		// D^-1
		b3Mat33 invD = b3SymInverse(D);
		link->m_invD = invD;

		// U * D^-1
		b3ForceVec U_invD[3];
		U_invD[0] = invD[0][0] * U[0] + invD[0][1] * U[1] + invD[0][2] * U[2];
		U_invD[1] = invD[1][0] * U[0] + invD[1][1] * U[1] + invD[1][2] * U[2];
		U_invD[2] = invD[2][0] * U[0] + invD[2][1] * U[1] + invD[2][2] * U[2];

		// I_a = I_A - U * D^-1 * U^T
		b3SpInertia M1 = b3Outer(U[0], U_invD[0]);
		b3SpInertia M2 = b3Outer(U[1], U_invD[1]);
		b3SpInertia M3 = b3Outer(U[2], U_invD[2]);

		b3SpInertia I_a = link->m_I_A;
		I_a -= M1;
		I_a -= M2;
		I_a -= M3;

		// u = tau - S^T * F_A
		u[0] = -b3Dot(S[0], F_A);
		u[1] = -b3Dot(S[1], F_A);
		u[2] = -b3Dot(S[2], F_A);

		// U * D^-1 * u
		b3ForceVec U_invD_u = U_invD[0] * u[0] + U_invD[1] * u[1] + U_invD[2] * u[2];
		
		// F_a = F_A + I_a * c + U * D^-1 * u
		b3ForceVec F_a = F_A + I_a * c + U_invD_u;

		b3SpInertia I_a_i = b3MulT(link->m_X_i_j, I_a);
		b3ForceVec F_a_i = b3MulT(link->m_X_i_j, F_a);

		parent->m_I_A += I_a_i;
		parent->m_F_A += F_a_i;
	}

	// Propagate down accelerations
	{
		b3RopeBody* body = m_links;

		if (body->m_m == 0.0f)
		{
			body->m_sa.SetZero();
		}
		else
		{
			// a = I^-1 * F 
			if (m_count == 1)
			{
				float32 inv_m = body->m_m > 0.0f ? 1.0f / body->m_m : 0.0f;
				float32 invI = body->m_I > 0.0f ? 1.0f / body->m_I : 0.0f;

				body->m_sa.w = -invI * body->m_F_A.f;
				body->m_sa.v = -inv_m * body->m_F_A.n;
			}
			else
			{
				body->m_sa = body->m_I_A.Solve(-body->m_F_A);
			}
		}
	}

	for (u32 j = 1; j < m_count; ++j)
	{
		b3RopeBody* link = m_links + j;
		b3RopeBody* parent = link - 1;
		b3MotionVec* S = link->m_S;
		b3MotionVec c = link->m_sc;
		b3ForceVec* U = link->m_U;
		b3Vec3 u = link->m_u;
		
		b3MotionVec parent_a = b3Mul(link->m_X_i_j, parent->m_sa);
		b3MotionVec a = parent_a + c;

		// u - U^T * a
		b3Vec3 b;
		b[0] = u[0] - b3Dot(a, U[0]);
		b[1] = u[1] - b3Dot(a, U[1]);
		b[2] = u[2] - b3Dot(a, U[2]);

		// D^-1 * b
		link->m_a = link->m_invD * b;
		
		b3MotionVec joint_a = S[0] * link->m_a[0] + S[1] * link->m_a[1] + S[2] * link->m_a[2];

		link->m_sa = a + joint_a;
	}
	
	// Integrate
	float32 max_w = 8.0f * 2.0f * B3_PI;
	b3Vec3 min(-max_w, -max_w, -max_w);
	b3Vec3 max(max_w, max_w, max_w);

	{
		b3RopeBody* b = m_links;

		// Convert local to global acceleration
		b3Vec3 v_dot = b3Mul(m_q, b->m_sa.v);
		b3Vec3 w_dot = b3Mul(m_q, b->m_sa.w);

		// Integrate acceleration
		m_v += h * v_dot;
		m_w += h * w_dot;
		
		// Integrate velocity		
		m_p += h * m_v;

		b3Quat q_w(m_w.x, m_w.y, m_w.z, 0.0f);
		b3Quat q_dot = 0.5f * q_w * m_q;
		m_q += h * q_dot;
		m_q.Normalize();
	}
	
	for (u32 i = 1; i < m_count; ++i)
	{
		b3RopeBody* link = m_links + i;
		
		// Integrate acceleration
		link->m_v += h * link->m_a;

		// Avoid numerical instability due to large velocities
		link->m_v = b3Clamp(link->m_v, min, max);

		// Integrate velocity		
		b3Quat q_w(link->m_v.x, link->m_v.y, link->m_v.z, 0.0f);
		b3Quat q_dot = 0.5f * link->m_p * q_w;

		link->m_p += h * q_dot;
		link->m_p.Normalize();
	}

	// Propagate down transforms
	m_links->m_X.rotation = b3QuatMat33(m_q);
	m_links->m_X.position = m_p;
	m_links->m_invX = b3Inverse(m_links->m_X);

	for (u32 j = 1; j < m_count; ++j)
	{
		b3RopeBody* link = m_links + j;
		b3RopeBody* parent = link - 1;

		b3Transform X_J = link->X_J();
		b3Transform X_i_j = link->m_X_J_j * X_J * link->m_X_i_J;

		link->m_invX = X_i_j * parent->m_invX;
		link->m_X = b3Inverse(link->m_invX);
	}
}

void b3Rope::Draw() const
{
	if (m_count == 0)
	{
		return;
	}

	{
		b3RopeBody* b = m_links;
		
		b3Draw_draw->DrawTransform(b->m_X);
		b3Draw_draw->DrawSolidSphere(b->m_X.position, 0.2f, b3Color_green);
	}

	for (u32 i = 1; i < m_count; ++i)
	{
		b3RopeBody* b = m_links + i;
		b3RopeBody* b0 = b - 1;

		b3Transform X_J = b0->m_X * b3Inverse(b->m_X_i_J);
		b3Transform X_J0 = b->m_X * b->m_X_J_j;
		
		b3Draw_draw->DrawTransform(X_J);
		b3Draw_draw->DrawPoint(X_J.position, 5.0f, b3Color_red);

		b3Draw_draw->DrawTransform(X_J0);
		b3Draw_draw->DrawPoint(X_J0.position, 5.0f, b3Color_red);

		b3Draw_draw->DrawTransform(b->m_X);
		b3Draw_draw->DrawSolidSphere(b->m_X.position, 0.2f, b3Color_green);
	}
}