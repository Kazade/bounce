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

#ifndef B3_SPARSE_MAT_33_PATTERN_H
#define B3_SPARSE_MAT_33_PATTERN_H

#include <bounce/common/math/mat33.h>

// An element in a sparse matrix.
struct b3RowValue
{
	u32 column;
	b3Mat33 value;
	b3RowValue* next;
};

// Singly linked list of row elements.
struct b3RowValueList
{
	b3RowValueList()
	{
		head = nullptr;
		count = 0;
	}

	~b3RowValueList() { }

	void PushFront(b3RowValue* link)
	{
		link->next = head;
		head = link;
		++count;
	}

	b3RowValue* head;
	u32 count;
};

// A sparse matrix pattern.
// Each row is a list of non-zero elements in the row.
// This class uses b3Alloc/b3Free and not the sparse allocator.
struct b3SparseMat33Pattern
{
	//
	b3SparseMat33Pattern();

	// 
	b3SparseMat33Pattern(u32 m);

	//
	~b3SparseMat33Pattern();

	// 
	b3Mat33* CreateElement(u32 i, u32 j);

	// 
	const b3Mat33& operator()(u32 i, u32 j) const;
	
	//
	void SetZero();

	u32 rowCount;
	b3RowValueList* rows;
};

inline b3SparseMat33Pattern::b3SparseMat33Pattern()
{
	rowCount = 0;
	rows = nullptr;
}

inline b3SparseMat33Pattern::b3SparseMat33Pattern(u32 m)
{
	rowCount = m;
	rows = (b3RowValueList*)b3Alloc(rowCount * sizeof(b3RowValueList));
	for (u32 i = 0; i < rowCount; ++i)
	{
		new (rows + i)b3RowValueList();
	}
}

inline b3SparseMat33Pattern::~b3SparseMat33Pattern()
{
	for (u32 i = 0; i < rowCount; ++i)
	{
		b3RowValueList* vs = rows + i;

		b3RowValue* v = vs->head;
		while (v)
		{
			b3RowValue* v0 = v->next;
			b3Free(v);
			v = v0;
		}

		vs->~b3RowValueList();
	}

	b3Free(rows);
}

inline const b3Mat33& b3SparseMat33Pattern::operator()(u32 i, u32 j) const
{
	B3_ASSERT(i < rowCount);
	B3_ASSERT(j < rowCount);

	b3RowValueList* vs = rows + i;

	for (b3RowValue* v = vs->head; v; v = v->next)
	{
		if (v->column == j)
		{
			return v->value;
		}
	}

	return b3Mat33_zero;
}

inline b3Mat33* b3SparseMat33Pattern::CreateElement(u32 i, u32 j)
{
	B3_ASSERT(i < rowCount);
	B3_ASSERT(j < rowCount);

	b3RowValueList* vs = rows + i;

	for (b3RowValue* v = vs->head; v; v = v->next)
	{
		if (v->column == j)
		{
			return &v->value;
		}
	}

	b3RowValue* v = (b3RowValue*)b3Alloc(sizeof(b3RowValue));
	v->column = j;
	v->value.SetZero();

	vs->PushFront(v);

	return &v->value;
}

inline void b3SparseMat33Pattern::SetZero()
{
	for (u32 i = 0; i < rowCount; ++i)
	{
		b3RowValueList* vs = rows + i;

		b3RowValue* v = vs->head;
		while (v)
		{
			v->value.SetZero();
			v = v->next;
		}
	}
}

#endif