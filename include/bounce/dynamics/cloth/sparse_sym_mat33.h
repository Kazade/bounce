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
	
	u32 M;
	u32* row_ptrs; 
	u32 value_capacity;
	u32 value_count;
	b3Mat33* values;
	u32* value_columns;
};

inline b3SparseSymMat33::b3SparseSymMat33()
{
	M = 0;
	row_ptrs = nullptr;
	value_count = 0;
	value_capacity = 0;
	values = nullptr;
	value_columns = nullptr;
}

inline b3SparseSymMat33::b3SparseSymMat33(u32 m)
{
	M = m;
	row_ptrs = (u32*)b3Alloc((M + 1) * sizeof(u32));
	memset(row_ptrs, 0, (M + 1) * sizeof(u32));
	value_count = 0;
	//value_capacity = M * (M + 1) / 2;
	value_capacity = 256;
	values = (b3Mat33*)b3Alloc(value_capacity * sizeof(b3Mat33));
	value_columns = (u32*)b3Alloc(value_capacity * sizeof(u32));
}

inline b3SparseSymMat33::b3SparseSymMat33(const b3SparseSymMat33& m)
{
	M = m.M;
	row_ptrs = (u32*)b3Alloc((M + 1) * sizeof(u32));
	memset(row_ptrs, 0, (M + 1) * sizeof(u32));
	value_count = m.value_count;
	value_capacity = m.value_capacity;
	values = (b3Mat33*)b3Alloc(value_capacity * sizeof(b3Mat33));
	value_columns = (u32*)b3Alloc(value_capacity * sizeof(u32));

	Copy(m);
}

inline b3SparseSymMat33::~b3SparseSymMat33()
{
	b3Free(value_columns);
	b3Free(values);
	b3Free(row_ptrs);
}

inline b3SparseSymMat33& b3SparseSymMat33::operator=(const b3SparseSymMat33& _m)
{
	if (_m.values == values)
	{
		return *this;
	}
	
	b3Free(row_ptrs);
	b3Free(values);
	b3Free(value_columns);

	M = _m.M;
	row_ptrs = (u32*)b3Alloc((M + 1) * sizeof(u32));
	value_count = _m.value_count;
	value_capacity = _m.value_capacity;
	values = (b3Mat33*)b3Alloc(value_capacity * sizeof(b3Mat33));
	value_columns = (u32*)b3Alloc(value_capacity * sizeof(u32));

	Copy(_m);

	return *this;
}

inline void b3SparseSymMat33::Copy(const b3SparseSymMat33& _m)
{
	B3_ASSERT(M == _m.M);
	B3_ASSERT(value_capacity == _m.value_capacity);
	B3_ASSERT(value_count == _m.value_count);

	memcpy(row_ptrs, _m.row_ptrs, (M + 1) * sizeof(u32));
	memcpy(values, _m.values, value_count * sizeof(b3Mat33));
	memcpy(value_columns, _m.value_columns, value_count * sizeof(u32));
}

inline const b3Mat33& b3SparseSymMat33::operator()(u32 i, u32 j) const
{
	B3_ASSERT(i < M);
	B3_ASSERT(j < M);

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

		if (row_value_column < j)
		{
			break;
		}

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
	B3_ASSERT(j < M);

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

		if (row_value_column < j)
		{
			break;
		}

		if (row_value_column == j)
		{
			return values[row_value_index];
		}
	}

	// Select insert position
	u32 row_value_position = 0;
	for (u32 row_value = 0; row_value < row_value_count; ++row_value)
	{
		u32 row_value_index = row_value_begin + row_value;
		u32 row_value_column = value_columns[row_value_index];
		
		if (row_value_column > j)
		{
			row_value_position = row_value;
			break;
		}
	}

	if (value_count == value_capacity)
	{
		b3Mat33* old_values = values;
		u32* old_columns = value_columns;

		value_capacity *= 2;
		values = (b3Mat33*)b3Alloc(value_capacity * sizeof(b3Mat33));
		value_columns = (u32*)b3Alloc(value_capacity * sizeof(u32));
		
		memcpy(values, old_values, value_count * sizeof(b3Mat33));
		memcpy(value_columns, old_columns, value_count * sizeof(u32));

		b3Free(old_values);
		b3Free(old_columns);
	}
	
	B3_ASSERT(value_count < value_capacity);

	// Shift the values
	for (u32 row_value = value_count; row_value > row_value_begin + row_value_position; --row_value)
	{
		values[row_value] = values[row_value - 1];
		value_columns[row_value] = value_columns[row_value - 1];
	}

	// Insert the value
	value_columns[row_value_begin + row_value_position] = j;
	values[row_value_begin + row_value_position].SetZero();
	++value_count;

	// Shift the row pointers 
	for (u32 row_ptr_index = i + 1; row_ptr_index < M + 1; ++row_ptr_index)
	{
		++row_ptrs[row_ptr_index];
	}

	// Return the inserted value
	return values[row_value_begin + row_value_position];
}

inline void b3SparseSymMat33::operator+=(const b3SparseSymMat33& m)
{
	B3_ASSERT(M == m.M);

	for (u32 row = 0; row < m.M; ++row)
	{
		u32 row_value_begin = m.row_ptrs[row];
		u32 row_value_count = m.row_ptrs[row + 1] - row_value_begin;

		for (u32 row_value = 0; row_value < row_value_count; ++row_value)
		{
			u32 row_value_index = row_value_begin + row_value;
			u32 row_value_column = m.value_columns[row_value_index];

			(*this)(row, row_value_column) += m.values[row_value_index];
		}
	}
}

inline void b3SparseSymMat33::operator-=(const b3SparseSymMat33& m)
{
	B3_ASSERT(M == m.M);

	for (u32 row = 0; row < m.M; ++row)
	{
		u32 row_value_begin = m.row_ptrs[row];
		u32 row_value_count = m.row_ptrs[row + 1] - row_value_begin;

		for (u32 row_value = 0; row_value < row_value_count; ++row_value)
		{
			u32 row_value_index = row_value_begin + row_value;
			u32 row_value_column = m.value_columns[row_value_index];

			(*this)(row, row_value_column) -= m.values[row_value_index];
		}
	}
}

inline void b3SparseSymMat33::Diagonal(b3DiagMat33& out) const
{
	B3_ASSERT(M == out.n);
	for (u32 row = 0; row < M; ++row)
	{
		out[row] = (*this)(row, row);
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
	B3_ASSERT(A.M == out.n);

	out.SetZero();

	for (u32 row = 0; row < A.M; ++row)
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

inline void b3Mul(b3SparseSymMat33& out, float32 A, const b3SparseSymMat33& B)
{
	B3_ASSERT(out.M == B.M);

	if (A == 0.0f)
	{
		return;
	}

	for (u32 row = 0; row < B.M; ++row)
	{
		u32 row_value_begin = B.row_ptrs[row];
		u32 row_value_count = B.row_ptrs[row + 1] - row_value_begin;

		for (u32 row_value = 0; row_value < row_value_count; ++row_value)
		{
			u32 row_value_index = row_value_begin + row_value;
			u32 row_value_column = B.value_columns[row_value_index];

			out(row, row_value_column) = A * B.values[row_value_index];
		}
	}
}

inline b3SparseSymMat33 operator+(const b3SparseSymMat33& A, const b3SparseSymMat33& B)
{
	b3SparseSymMat33 result(A.M);
	b3Add(result, A, B);
	return result;
}

inline b3SparseSymMat33 operator-(const b3SparseSymMat33& A, const b3SparseSymMat33& B)
{
	b3SparseSymMat33 result(A.M);
	b3Sub(result, A, B);
	return result;
}

inline b3SparseSymMat33 operator*(float32 A, const b3SparseSymMat33& B)
{
	b3SparseSymMat33 result(B.M);
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