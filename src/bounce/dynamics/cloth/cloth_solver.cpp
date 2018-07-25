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

#include <bounce/dynamics/cloth/cloth_solver.h>
#include <bounce/dynamics/cloth/cloth.h>
#include <bounce/dynamics/cloth/particle.h>
#include <bounce/dynamics/cloth/force.h>
#include <bounce/dynamics/cloth/dense_vec3.h>
#include <bounce/dynamics/cloth/diag_mat33.h>
#include <bounce/dynamics/cloth/sparse_sym_mat33.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/memory/stack_allocator.h>

// Here, we solve Ax = b using the Modified Preconditioned Conjugate Gradient (MPCG) algorithm.
// described in the paper:
// "Large Steps in Cloth Simulation - David Baraff, Andrew Witkin".

// Some improvements for the original MPCG algorithm are described in the paper:
// "On the modified conjugate gradient method in cloth simulation - Uri M. Ascher, Eddy Boxerman".

u32 b3_clothSolverIterations = 0;

b3ClothSolver::b3ClothSolver(const b3ClothSolverDef& def)
{
	m_allocator = def.stack;

	m_particleCapacity = def.particleCapacity;
	m_particleCount = 0;
	m_particles = (b3Particle**)m_allocator->Allocate(m_particleCapacity * sizeof(b3Particle*));

	m_forceCapacity = def.forceCapacity;
	m_forceCount = 0;
	m_forces = (b3Force**)m_allocator->Allocate(m_forceCapacity * sizeof(b3Force*));;

	m_contactCapacity = def.contactCapacity;
	m_contactCount = 0;
	m_contacts = (b3BodyContact**)m_allocator->Allocate(m_contactCapacity * sizeof(b3BodyContact*));

	m_constraintCapacity = def.particleCapacity;
	m_constraintCount = 0;
	m_constraints = (b3AccelerationConstraint*)m_allocator->Allocate(m_constraintCapacity * sizeof(b3AccelerationConstraint));

	m_velocityConstraints = (b3ClothContactVelocityConstraint*)m_allocator->Allocate(m_contactCount * sizeof(b3ClothContactVelocityConstraint));
}

b3ClothSolver::~b3ClothSolver()
{
	m_allocator->Free(m_velocityConstraints);
	m_allocator->Free(m_constraints);
	m_allocator->Free(m_contacts);
	m_allocator->Free(m_forces);
	m_allocator->Free(m_particles);
}

void b3ClothSolver::Add(b3Particle* p)
{
	p->m_solverId = m_particleCount;
	m_particles[m_particleCount++] = p;
}

void b3ClothSolver::Add(b3BodyContact* c)
{
	m_contacts[m_contactCount++] = c;
}

void b3ClothSolver::Add(b3Force* f)
{
	m_forces[m_forceCount++] = f;
}

void b3ClothSolver::ApplyForces()
{
	for (u32 i = 0; i < m_forceCount; ++i)
	{
		m_forces[i]->Apply(&m_solverData);
	}
}

void b3AccelerationConstraint::Apply(const b3ClothSolverData* data)
{
	b3DiagMat33& sS = *data->S;
	b3DenseVec3& sz = *data->z;

	sz[i1] = z;

	b3Mat33 I; I.SetIdentity();

	switch (ndof)
	{
	case 3:
	{
		sS[i1] = I;
		break;
	}
	case 2:
	{
		sS[i1] = I - b3Outer(p, p);
		break;
	}
	case 1:
	{
		sS[i1] = I - b3Outer(p, p) - b3Outer(q, q);
		break;
	}
	case 0:
	{
		sS[i1].SetZero();
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	}
}

void b3ClothSolver::ApplyConstraints()
{
	b3DiagMat33& S = *m_solverData.S;
	b3DenseVec3& z = *m_solverData.z;

	S.SetIdentity();
	z.SetZero();

	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Particle* p = m_particles[i];
		if (p->m_type != e_dynamicParticle)
		{
			b3AccelerationConstraint* ac = m_constraints + m_constraintCount;
			++m_constraintCount;
			ac->i1 = i;
			ac->ndof = 0;
			ac->z.SetZero();
		}
	}

	for (u32 i = 0; i < m_constraintCount; ++i)
	{
		m_constraints[i].Apply(&m_solverData);
	}
}

void b3ClothSolver::Solve(float32 dt, const b3Vec3& gravity)
{
	b3DenseVec3 sx(m_particleCount);
	b3DenseVec3 sv(m_particleCount);
	b3DenseVec3 sf(m_particleCount);
	b3DenseVec3 sy(m_particleCount);
	b3DenseVec3 sx0(m_particleCount);
	b3SparseSymMat33 dfdx(m_particleCount);
	b3SparseSymMat33 dfdv(m_particleCount);
	b3DiagMat33 S(m_particleCount);
	b3DenseVec3 z(m_particleCount);

	m_solverData.x = &sx;
	m_solverData.v = &sv;
	m_solverData.f = &sf;
	m_solverData.y = &sy;
	m_solverData.dfdx = &dfdx;
	m_solverData.dfdv = &dfdv;
	m_solverData.S = &S;
	m_solverData.z = &z;
	m_solverData.dt = dt;
	m_solverData.invdt = 1.0f / dt;

	float32 h = dt;

	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Particle* p = m_particles[i];

		sx[i] = p->m_position;
		sv[i] = p->m_velocity;
		sf[i] = p->m_force;

		// Apply weight
		if (p->m_type == e_dynamicParticle)
		{
			sf[i] += p->m_mass * gravity;
		}

		sy[i] = p->m_translation;
		sx0[i] = p->m_x;
	}

	// Apply internal translations
	for (u32 i = 0; i < m_contactCount; ++i)
	{
		b3BodyContact* c = m_contacts[i];
		b3Particle* p = c->p1;

		if (p->m_type == e_dynamicParticle)
		{
			b3Vec3 dx = c->p - p->m_position;
			sy[p->m_solverId] += dx;
		}
	}

	// Integrate velocities
	
	// Apply internal forces
	ApplyForces();

	// Apply constraints
	ApplyConstraints();

	// Solve Ax = b, where
	// A = M - h * dfdv - h * h * dfdx
	// b = h * (f0 + h * dfdx * v0 + dfdx * y) 

	// A
	b3SparseSymMat33 A(m_particleCount);
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		A(i, i) = b3Diagonal(m_particles[i]->m_mass);
	}
	A += -h * dfdv - h * h * dfdx;

	// b
	b3DenseVec3 b = h * (sf + h * (dfdx * sv) + dfdx * sy);

	// x
	b3DenseVec3 x(m_particleCount);
	u32 iterations = 0;

	Solve(x, iterations, A, b, S, z, sx0);
	b3_clothSolverIterations = iterations;

	sv = sv + x;

	// Initialize velocity constraints
	InitializeVelocityConstraints();

	// Warm start velocity constraints
	WarmStart();

	// Solve velocity constraints
	const u32 kVelocityIterations = 10;
	for (u32 i = 0; i < kVelocityIterations; ++i)
	{
		SolveVelocityConstraints();
	}

	// Integrate positions
	sx = sx + h * sv + sy;

	// Copy state buffers back to the particles
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Particle* p = m_particles[i];

		p->m_position = sx[i];
		p->m_velocity = sv[i];

		// Cache x to improve convergence
		p->m_x = x[i];
	}
}

void b3ClothSolver::Solve(b3DenseVec3& x, u32& iterations,
	const b3SparseSymMat33& A, const b3DenseVec3& b, const b3DiagMat33& S, const b3DenseVec3& z, const b3DenseVec3& y) const
{
	B3_PROFILE("Solve Ax = b");

	// P = diag(A)
	b3DiagMat33 inv_P(m_particleCount);
	A.Diagonal(inv_P);

	// Invert
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Mat33& D = inv_P[i];

		// Sylvester Criterion to ensure PD-ness
		B3_ASSERT(b3Det(D.x, D.y, D.z) > 0.0f);

		B3_ASSERT(D.x.x != 0.0f);
		float32 xx = 1.0f / D.x.x;

		B3_ASSERT(D.y.y != 0.0f);
		float32 yy = 1.0f / D.y.y;

		B3_ASSERT(D.z.z != 0.0f);
		float32 zz = 1.0f / D.z.z;

		D = b3Diagonal(xx, yy, zz);
	}

	// I
	b3DiagMat33 I(m_particleCount);
	I.SetIdentity();

	// x = S * y + (I - S) * z 
	x = (S * y) + (I - S) * z;

	// b^ = S * (b - A * (I - S) * z)
	b3DenseVec3 b_hat = S * (b - A * ((I - S) * z));

	// b_delta = dot(b^, P^-1 * b_^)
	float32 b_delta = b3Dot(b_hat, inv_P * b_hat);

	// r = S * (b - A * x)
	b3DenseVec3 r = S * (b - A * x);

	// p = S * (P^-1 * r)
	b3DenseVec3 p = S * (inv_P * r);

	// delta_new = dot(r, p)
	float32 delta_new = b3Dot(r, p);

	// Set the tolerance.
	const float32 tolerance = 10.0f * B3_EPSILON;

	// Maximum number of iterations.
	// Stop at this iteration if diverged.
	const u32 max_iterations = 20;

	u32 iteration = 0;

	// Main iteration loop.
	for (;;)
	{
		// Divergence check.
		if (iteration >= max_iterations)
		{
			break;
		}

		// Convergence check.
		if (delta_new <= tolerance * tolerance * b_delta)
		{
			break;
		}

		// s = S * (A * p)
		b3DenseVec3 s = S * (A * p);

		// alpha = delta_new / dot(p, s)
		float32 alpha = delta_new / b3Dot(p, s);

		// x = x + alpha * p
		x = x + alpha * p;

		// r = r - alpha * s
		r = r - alpha * s;

		// h = inv_P * r
		b3DenseVec3 h = inv_P * r;

		// delta_old = delta_new
		float32 delta_old = delta_new;

		// delta_new = dot(r, h)
		delta_new = b3Dot(r, h);

		// beta = delta_new / delta_old
		float32 beta = delta_new / delta_old;

		// p = S * (h + beta * p)
		p = S * (h + beta * p);

		++iteration;
	}

	iterations = iteration;
}

void b3ClothSolver::InitializeVelocityConstraints()
{
	b3DenseVec3& x = *m_solverData.x;
	b3DenseVec3& v = *m_solverData.v;

	for (u32 i = 0; i < m_contactCount; ++i)
	{
		b3BodyContact* c = m_contacts[i];
		b3ClothContactVelocityConstraint* vc = m_velocityConstraints + i;

		vc->indexA = c->p1->m_solverId;
		vc->bodyB = c->s2->GetBody();

		vc->invMassA = c->p1->m_type == e_staticParticle ? 0.0f : c->p1->m_invMass;
		vc->invMassB = vc->bodyB->GetInverseMass();

		vc->invIA.SetZero();
		vc->invIB = vc->bodyB->GetWorldInverseInertia();

		vc->friction = c->s2->GetFriction();
	}

	for (u32 i = 0; i < m_contactCount; ++i)
	{
		b3BodyContact* c = m_contacts[i];
		b3ClothContactVelocityConstraint* vc = m_velocityConstraints + i;

		u32 indexA = vc->indexA;
		b3Body* bodyB = vc->bodyB;

		float32 mA = vc->invMassA;
		float32 mB = vc->invMassB;

		b3Mat33 iA = vc->invIA;
		b3Mat33 iB = vc->invIB;

		b3Vec3 xA = x[indexA];
		b3Vec3 xB = bodyB->GetPosition();

		// Ensure the normal points from shape 1 to the shape 2
		vc->normal = -c->n;
		vc->tangent1 = c->t1;
		vc->tangent2 = c->t2;
		vc->point = c->p;

		b3Vec3 point = c->p;

		b3Vec3 rA = point - xA;
		b3Vec3 rB = point - xB;

		vc->rA = rA;
		vc->rB = rB;
		
		vc->normalImpulse = c->normalImpulse;
		vc->tangentImpulse = c->tangentImpulse;
		vc->motorImpulse = c->motorImpulse;

		{
			b3Vec3 n = vc->normal;
			
			b3Vec3 rnA = b3Cross(rA, n);
			b3Vec3 rnB = b3Cross(rB, n);

			float32 K = mA + mB + b3Dot(iA * rnA, rnA) + b3Dot(iB * rnB, rnB);

			vc->normalMass = K > 0.0f ? 1.0f / K : 0.0f;
		}

		{
			b3Vec3 t1 = vc->tangent1;
			b3Vec3 t2 = vc->tangent2;

			b3Vec3 rn1A = b3Cross(rA, t1);
			b3Vec3 rn1B = b3Cross(rB, t1);
			b3Vec3 rn2A = b3Cross(rA, t2);
			b3Vec3 rn2B = b3Cross(rB, t2);

			// dot(t1, t2) = 0
			// J1_l1 * M1 * J2_l1 = J1_l2 * M2 * J2_l2 = 0
			float32 k11 = mA + mB + b3Dot(iA * rn1A, rn1A) + b3Dot(iB * rn1B, rn1B);
			float32 k12 = b3Dot(iA * rn1A, rn2A) + b3Dot(iB * rn1B, rn2B);
			float32 k22 = mA + mB + b3Dot(iA * rn2A, rn2A) + b3Dot(iB * rn2B, rn2B);

			b3Mat22 K;
			K.x.Set(k11, k12);
			K.y.Set(k12, k22);

			vc->tangentMass = b3Inverse(K);
		}

		{
			float32 mass = b3Dot(vc->normal, (iA + iB) * vc->normal);
			vc->motorMass = mass > 0.0f ? 1.0f / mass : 0.0f;
		}
	}
}

void b3ClothSolver::WarmStart()
{
	b3DenseVec3& v = *m_solverData.v;

	for (u32 i = 0; i < m_contactCount; ++i)
	{
		b3ClothContactVelocityConstraint* vc = m_velocityConstraints + i;
		
		u32 indexA = vc->indexA;
		b3Body* bodyB = vc->bodyB;

		b3Vec3 vA = v[indexA];
		b3Vec3 vB = bodyB->GetLinearVelocity();

		b3Vec3 wA; wA.SetZero();
		b3Vec3 wB = bodyB->GetAngularVelocity();
		
		float32 mA = vc->invMassA;
		float32 mB = vc->invMassB;

		b3Mat33 iA = vc->invIA;
		b3Mat33 iB = vc->invIB;

		b3Vec3 P = vc->normalImpulse * vc->normal;

		vA -= mA * P;
		wA -= iA * b3Cross(vc->rA, P);

		vB += mB * P;
		wB += iB * b3Cross(vc->rB, P);

		b3Vec3 P1 = vc->tangentImpulse.x * vc->tangent1;
		b3Vec3 P2 = vc->tangentImpulse.y * vc->tangent2;
		b3Vec3 P3 = vc->motorImpulse * vc->normal;

		vA -= mA * (P1 + P2);
		wA -= iA * (b3Cross(vc->rA, P1 + P2) + P3);

		vB += mB * (P1 + P2);
		wB += iB * (b3Cross(vc->rB, P1 + P2) + P3);

		v[indexA] = vA;

		bodyB->SetLinearVelocity(vB);
		bodyB->SetAngularVelocity(wB);
	}
}

void b3ClothSolver::SolveVelocityConstraints()
{
	b3DenseVec3& v = *m_solverData.v;

	for (u32 i = 0; i < m_contactCount; ++i)
	{
		b3ClothContactVelocityConstraint* vc = m_velocityConstraints + i;

		u32 indexA = vc->indexA;
		b3Body* bodyB = vc->bodyB;

		b3Vec3 vA = v[indexA];
		b3Vec3 vB = bodyB->GetLinearVelocity();

		b3Vec3 wA; wA.SetZero();
		b3Vec3 wB = bodyB->GetAngularVelocity();

		float32 mA = vc->invMassA;
		float32 mB = vc->invMassB;

		b3Mat33 iA = vc->invIA;
		b3Mat33 iB = vc->invIB;

		// Ensure the normal points from shape 1 to the shape 2
		b3Vec3 normal = vc->normal;
		b3Vec3 point = vc->point;

		b3Vec3 rA = vc->rA;
		b3Vec3 rB = vc->rB;

		// Solve normal constraint.
		{
			b3Vec3 rnA = b3Cross(rA, normal);
			b3Vec3 rnB = b3Cross(rB, normal);

			float32 K = mA + mB + b3Dot(iA * rnA, rnA) + b3Dot(iB * rnB, rnB);

			float32 invK = K > 0.0f ? 1.0f / K : 0.0f;

			b3Vec3 dv = vB + b3Cross(wB, rB) - vA - b3Cross(wA, rA);
			float32 Cdot = b3Dot(normal, dv);

			float32 impulse = -invK * Cdot;

			float32 oldImpulse = vc->normalImpulse;
			vc->normalImpulse = b3Max(vc->normalImpulse + impulse, 0.0f);
			impulse = vc->normalImpulse - oldImpulse;

			b3Vec3 P = impulse * normal;

			vA -= mA * P;
			wA -= iA * b3Cross(rA, P);

			vB += mB * P;
			wB += iB * b3Cross(rB, P);
		}

		// Solve tangent constraints.
		{
			b3Vec3 dv = vB + b3Cross(wB, rB) - vA - b3Cross(wA, rA);

			b3Vec2 Cdot;
			Cdot.x = b3Dot(dv, vc->tangent1);
			Cdot.y = b3Dot(dv, vc->tangent2);

			b3Vec2 impulse = vc->tangentMass * -Cdot;
			b3Vec2 oldImpulse = vc->tangentImpulse;
			vc->tangentImpulse += impulse;

			float32 maxImpulse = vc->friction * vc->normalImpulse;
			if (b3Dot(vc->tangentImpulse, vc->tangentImpulse) > maxImpulse * maxImpulse)
			{
				vc->tangentImpulse.Normalize();
				vc->tangentImpulse *= maxImpulse;
			}

			impulse = vc->tangentImpulse - oldImpulse;

			b3Vec3 P1 = impulse.x * vc->tangent1;
			b3Vec3 P2 = impulse.y * vc->tangent2;
			b3Vec3 P = P1 + P2;

			vA -= mA * P;
			wA -= iA * b3Cross(rA, P);

			vB += mB * P;
			wB += iB * b3Cross(rB, P);
		}

		// Solve motor constraint.
		{
			float32 Cdot = b3Dot(vc->normal, wB - wA);
			float32 impulse = -vc->motorMass * Cdot;
			float32 oldImpulse = vc->motorImpulse;
			float32 maxImpulse = vc->friction * vc->normalImpulse;
			vc->motorImpulse = b3Clamp(vc->motorImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = vc->motorImpulse - oldImpulse;

			b3Vec3 P = impulse * vc->normal;

			wA -= iA * P;
			wB += iB * P;
		}

		v[indexA] = vA;

		bodyB->SetLinearVelocity(vB);
		bodyB->SetAngularVelocity(wB);
	}
}

void b3ClothSolver::StoreImpulses()
{
	for (u32 i = 0; i < m_contactCount; ++i)
	{
		b3BodyContact* c = m_contacts[i];
		b3ClothContactVelocityConstraint* vc = m_velocityConstraints + i;

		c->normalImpulse = vc->normalImpulse;
		c->tangentImpulse = vc->tangentImpulse;
		c->motorImpulse = vc->motorImpulse;
	}
}