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

#include <bounce/common/math/mat33.h>
#include <bounce/dynamics/cloth/diag_mat33.h>
#include <bounce/dynamics/cloth/dense_vec3.h>

struct b3SparseSymMat33
{
	// 
	b3SparseSymMat33(u32 m, u32 n);
	
	//
	~b3SparseSymMat33();

	// 
	b3Mat33& operator()(u32 i, u32 j);

	// 
	const b3Mat33& operator()(u32 i, u32 j) const;

	// 
	void Diagonal(b3DiagMat33& out) const;

	// 
	void SetZero();

	u32 M;
	u32 N;
	u32* row_ptrs; 
	u32 value_capacity;
	u32 value_count;
	b3Mat33* values;
	u32* value_columns;
};

inline b3SparseSymMat33::b3SparseSymMat33(u32 m, u32 n)
{
	B3_ASSERT(m == n);
	M = m;
	N = n;
	row_ptrs = (u32*)b3Alloc((M + 1) * sizeof(u32));
	memset(row_ptrs, 0, (M + 1) * sizeof(u32));
	value_count = 0;
	value_capacity = M * (M + 1) / 2;
	values = (b3Mat33*)b3Alloc(value_capacity * sizeof(b3Mat33));
	value_columns = (u32*)b3Alloc(value_capacity * sizeof(u32));
}

inline b3SparseSymMat33::~b3SparseSymMat33()
{
	b3Free(value_columns);
	b3Free(values);
	b3Free(row_ptrs);
}

inline const b3Mat33& b3SparseSymMat33::operator()(u32 i, u32 j) const
{
	B3_ASSERT(i < M);
	B3_ASSERT(j < N);

	// Ensure i, and j is on the upper triangle 
	if (i > j)
	{
		b3Swap(i, j);
	}

	u32 row_value_begin = row_ptrs[i];
	u32 row_value_count = row_ptrs[i + 1] - row_value_begin;

	for (u32 row_value = 0; row_value < row_value_count; ++row_value)
	{
		u32 row_value_index = row_value_begin + row_value;
		u32 row_value_column = value_columns[row_value_index];

		if (row_value_column == j)
		{
			return values[row_value_index];
		}
	}

	return b3Mat33_zero;
}

inline b3Mat33& b3SparseSymMat33::operator()(u32 i, u32 j)
{	
	B3_ASSERT(i < M);
	B3_ASSERT(j < N);

	// Ensure i, and j is on the upper triangle 
	if (i > j)
	{
		b3Swap(i, j);
	}

	u32 row_value_begin = row_ptrs[i];
	u32 row_value_count = row_ptrs[i + 1] - row_value_begin;

	for (u32 row_value = 0; row_value < row_value_count; ++row_value)
	{
		u32 row_value_index = row_value_begin + row_value;
		u32 row_value_column = value_columns[row_value_index];
		
		if (row_value_column == j)
		{
			return values[row_value_index];
		}
	}

	// Find insert position
	u32 row_value_k = 0;
	for (u32 row_value = 0; row_value < row_value_count; ++row_value)
	{
		u32 row_value_index = row_value_begin + row_value;
		u32 row_value_column = value_columns[row_value_index];
		
		if (row_value_column >= j)
		{
			row_value_k = row_value;
			break;
		}
	}

	// Shift the values
	u32 right_count = value_count - row_value_begin - row_value_k;
	memcpy(value_columns + row_value_begin + row_value_k + 1, value_columns + row_value_begin + row_value_k, right_count * sizeof(u32));
	memcpy(values + row_value_begin + row_value_k + 1, values + row_value_begin + row_value_k, right_count * sizeof(b3Mat33));

	// Shift the row pointers 
	for (u32 row_ptr_index = i + 1; row_ptr_index < M + 1; ++row_ptr_index)
	{
		++row_ptrs[row_ptr_index];
	}

	// Insert the value
	value_columns[row_value_begin + row_value_k] = j;
	++value_count;

	// Return the value
	return values[row_value_begin + row_value_k];
}

inline void b3SparseSymMat33::Diagonal(b3DiagMat33& out) const
{
	B3_ASSERT(N == out.n);

	for (u32 row = 0; row < M; ++row)
	{
		out[row].SetZero();

		u32 row_value_begin = row_ptrs[row];
		u32 row_value_count = row_ptrs[row + 1] - row_value_begin;

		for (u32 row_value = 0; row_value < row_value_count; ++row_value)
		{
			u32 row_value_index = row_value_begin + row_value;
			u32 row_value_column = value_columns[row_value_index];

			if (row == row_value_column)
			{
				out[row] = values[row_value_index];
				break;
			}
		}
	}
}

inline void b3SparseSymMat33::SetZero()
{
	for (u32 i = 0; i < M; ++i)
	{
		for (u32 j = i; j < M; ++j)
		{
			(*this)(i, j).SetZero();
		}
	}
}

inline void b3Mul(b3DenseVec3& out, const b3SparseSymMat33& A, const b3DenseVec3& v)
{
	B3_ASSERT(A.N == out.n);

	out.SetZero();

	for (u32 row = 0; row < A.N; ++row)
	{
		u32 row_value_begin = A.row_ptrs[row];
		u32 row_value_count = A.row_ptrs[row + 1] - row_value_begin;

		for (u32 row_value = 0; row_value < row_value_count; ++row_value)
		{
			u32 row_value_index = row_value_begin + row_value;

			u32 row_value_column = A.value_columns[row_value_index];

			out[row] += A.values[row_value_index] * v[row_value_column];

			if (row != row_value_column)
			{
				// A(i, j) == A(j, i)
				out[row_value_column] += A.values[row_value_index] * v[row];
			}
		}
	}
}

inline b3DenseVec3 operator*(const b3SparseSymMat33& A, const b3DenseVec3& v)
{
	b3DenseVec3 result(v.n);
	b3Mul(result, A, v);
	return result;
}

#endif