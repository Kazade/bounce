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

#ifndef B3_SPARSE_SYM_MAT_33_H
#define B3_SPARSE_SYM_MAT_33_H

#include <bounce/common/template/list.h>
#include <bounce/common/math/mat33.h>
#include <bounce/dynamics/cloth/diag_mat33.h>
#include <bounce/dynamics/cloth/dense_vec3.h>

// An element in a sparse symmetric matrix.
struct b3RowValue
{
	u32 column;
	b3Mat33 value;

	b3RowValue* m_prev;
	b3RowValue* m_next;
};

// A sparse symmetric matrix.
// Each row is a list of non-zero elements in the row.
// The total matrix capacity is bounded by 
// M * (M + 1) / 2
struct b3SparseSymMat33
{
	//
	b3SparseSymMat33();

	// 
	b3SparseSymMat33(u32 m);

	//
	b3SparseSymMat33(const b3SparseSymMat33& _m);

	//
	~b3SparseSymMat33();

	//
	b3SparseSymMat33& operator=(const b3SparseSymMat33& _m);

	// 
	void Copy(const b3SparseSymMat33& _m);

	// 
	b3Mat33& operator()(u32 i, u32 j);

	// 
	const b3Mat33& operator()(u32 i, u32 j) const;

	// 
	void operator+=(const b3SparseSymMat33& m);

	// 
	void operator-=(const b3SparseSymMat33& m);

	// 
	void Diagonal(b3DiagMat33& out) const;

	u32 rowCount;
	b3List2<b3RowValue>* rows; 
};

inline b3SparseSymMat33::b3SparseSymMat33()
{
	rowCount = 0;
	rows = nullptr;
}

inline b3SparseSymMat33::b3SparseSymMat33(u32 m)
{
	rowCount = m;
	rows = (b3List2<b3RowValue>*)b3Alloc(rowCount * sizeof(b3List2<b3RowValue>));
	for (u32 i = 0; i < rowCount; ++i)
	{
		new (rows + i)b3List2<b3RowValue>();
	}
}

inline b3SparseSymMat33::b3SparseSymMat33(const b3SparseSymMat33& m)
{
	rowCount = m.rowCount;
	rows = (b3List2<b3RowValue>*)b3Alloc(rowCount * sizeof(b3List2<b3RowValue>));
	for (u32 i = 0; i < rowCount; ++i)
	{
		new (rows + i)b3List2<b3RowValue>();
	}

	Copy(m);
}

inline b3SparseSymMat33::~b3SparseSymMat33()
{
	for (u32 i = 0; i < rowCount; ++i)
	{
		b3List2<b3RowValue>* vs = rows + i;

		b3RowValue* v = vs->m_head;
		while (v)
		{
			b3RowValue* v0 = v->m_next;
			b3Free(v);
			v = v0;
		}

		vs->~b3List2();
	}

	b3Free(rows);
}

inline b3SparseSymMat33& b3SparseSymMat33::operator=(const b3SparseSymMat33& _m)
{
	if (_m.rows == rows)
	{
		return *this;
	}
	
	b3Free(rows);

	rowCount = _m.rowCount;
	rows = (b3List2<b3RowValue>*)b3Alloc(rowCount * sizeof(b3List2<b3RowValue>));
	for (u32 i = 0; i < rowCount; ++i)
	{
		new (rows + i)b3List2<b3RowValue>();
	}

	Copy(_m);

	return *this;
}

inline void b3SparseSymMat33::Copy(const b3SparseSymMat33& _m)
{
	B3_ASSERT(rowCount == _m.rowCount);

	for (u32 row = 0; row < rowCount; ++row)
	{
		b3List2<b3RowValue>* vs1 = _m.rows + row;
		b3List2<b3RowValue>* vs2 = rows + row;

		B3_ASSERT(vs2->m_count == 0);

		for (b3RowValue* v1 = vs1->m_head; v1; v1 = v1->m_next)
		{
			b3RowValue* v2 = (b3RowValue*)b3Alloc(sizeof(b3RowValue));

			v2->column = v1->column;
			v2->value = v1->value;

			vs2->PushFront(v2);
		}
	}
}

inline const b3Mat33& b3SparseSymMat33::operator()(u32 i, u32 j) const
{
	B3_ASSERT(i < rowCount);
	B3_ASSERT(j < rowCount);

	// Ensure i, and j is on the upper triangle 
	if (i > j)
	{
		b3Swap(i, j);
	}

	b3List2<b3RowValue>* vs = rows + i;

	for (b3RowValue* v = vs->m_head; v; v = v->m_next)
	{
		u32 column = v->column;
		
		if (column < j)
		{
			break;
		}

		if (column == i)
		{
			return v->value;
		}
	}

	return b3Mat33_zero;
}

inline b3Mat33& b3SparseSymMat33::operator()(u32 i, u32 j)
{	
	B3_ASSERT(i < rowCount);
	B3_ASSERT(j < rowCount);

	// Ensure i, and j is on the upper triangle 
	if (i > j)
	{
		b3Swap(i, j);
	}

	b3List2<b3RowValue>* vs = rows + i;

	for (b3RowValue* v = vs->m_head; v; v = v->m_next)
	{
		u32 column = v->column;

		if (column < j)
		{
			break;
		}

		if (column == i)
		{
			return v->value;
		}
	}

	b3RowValue* v1 = (b3RowValue*)b3Alloc(sizeof(b3RowValue));
	v1->column = j;
	v1->value.SetZero();

	b3RowValue* v0 = nullptr;

	for (b3RowValue* v = vs->m_head; v; v = v->m_next)
	{
		u32 column = v->column;

		if (column > j)
		{
			v0 = v;
			break;
		}
	}

	if (v0 == nullptr)
	{
		vs->PushFront(v1);
	}
	else
	{
		vs->PushAfter(v0, v1);
	}

	return v1->value;
}

inline void b3SparseSymMat33::operator+=(const b3SparseSymMat33& m)
{
	B3_ASSERT(rowCount == m.rowCount);

	for (u32 i = 0; i < m.rowCount; ++i)
	{
		b3List2<b3RowValue>* mvs = m.rows + i;

		for (b3RowValue* v = mvs->m_head; v; v = v->m_next)
		{
			u32 j = v->column;

			(*this)(i, j) += v->value;
		}
	}
}

inline void b3SparseSymMat33::operator-=(const b3SparseSymMat33& m)
{
	B3_ASSERT(rowCount == m.rowCount);

	for (u32 i = 0; i < m.rowCount; ++i)
	{
		b3List2<b3RowValue>* mvs = m.rows + i;

		for (b3RowValue* v = mvs->m_head; v; v = v->m_next)
		{
			u32 j = v->column;

			(*this)(i, j) -= v->value;
		}
	}
}

inline void b3SparseSymMat33::Diagonal(b3DiagMat33& out) const
{
	B3_ASSERT(rowCount == out.n);
	
	for (u32 i = 0; i < rowCount; ++i)
	{
		out[i] = (*this)(i, i);
	}
}

inline void b3Add(b3SparseSymMat33& out, const b3SparseSymMat33& a, const b3SparseSymMat33& b)
{
	out = a;
	out += b;
}

inline void b3Sub(b3SparseSymMat33& out, const b3SparseSymMat33& a, const b3SparseSymMat33& b)
{
	out = a;
	out -= b;
}

inline void b3Mul(b3DenseVec3& out, const b3SparseSymMat33& A, const b3DenseVec3& v)
{
	B3_ASSERT(A.rowCount == out.n);

	out.SetZero();

	for (u32 i = 0; i < A.rowCount; ++i)
	{
		b3List2<b3RowValue>* vs = A.rows + i;

		for (b3RowValue* vA = vs->m_head; vA; vA = vA->m_next)
		{
			u32 j = vA->column;
			b3Mat33 a = vA->value;

			out[i] += a * v[j];

			if (i != j)
			{
				// a_ij == a_ji
				out[j] += a * v[i];
			}
		}
	}
}

inline void b3Mul(b3SparseSymMat33& out, float32 s, const b3SparseSymMat33& B)
{
	B3_ASSERT(out.rowCount == B.rowCount);

	if (s == 0.0f)
	{
		return;
	}

	for (u32 i = 0; i < B.rowCount; ++i)
	{
		b3List2<b3RowValue>* vs = B.rows + i;

		for (b3RowValue* vB = vs->m_head; vB; vB = vB->m_next)
		{
			u32 j = vB->column;
			b3Mat33 b = vB->value;

			out(i, j) = s * b;
		}
	}
}

inline b3SparseSymMat33 operator+(const b3SparseSymMat33& A, const b3SparseSymMat33& B)
{
	b3SparseSymMat33 result(A.rowCount);
	b3Add(result, A, B);
	return result;
}

inline b3SparseSymMat33 operator-(const b3SparseSymMat33& A, const b3SparseSymMat33& B)
{
	b3SparseSymMat33 result(A.rowCount);
	b3Sub(result, A, B);
	return result;
}

inline b3SparseSymMat33 operator*(float32 A, const b3SparseSymMat33& B)
{
	b3SparseSymMat33 result(B.rowCount);
	b3Mul(result, A, B);
	return result;
}

inline b3DenseVec3 operator*(const b3SparseSymMat33& A, const b3DenseVec3& v)
{
	b3DenseVec3 result(v.n);
	b3Mul(result, A, v);
	return result;
}

#endif