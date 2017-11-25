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

#ifndef B3_OBJECT_ARRAY_H
#define B3_OBJECT_ARRAY_H

#include <bounce/common/settings.h>

// An array for objects.
template <typename T>
class b3ObjectArray
{
public:
	const T& operator[](u32 i) const
	{
		B3_ASSERT(i < m_count);
		return m_elements[i];
	}

	T& operator[](u32 i)
	{
		B3_ASSERT(i < m_count);
		return m_elements[i];
	}

	const T* Get(u32 i) const
	{
		B3_ASSERT(i < m_count);
		return m_elements + i;
	}

	T* Get(u32 i)
	{
		B3_ASSERT(i < m_count);
		return m_elements + i;
	}

	const T* Elements() const
	{
		return m_elements;
	}

	T* Elements()
	{
		return m_elements;
	}

	void PushBack(const T& ele)
	{
		if (m_count == m_capacity)
		{
			// There is no capacity for one more element.
			T* oldElements = m_elements;
			m_capacity *= 2;
			m_elements = (T*)b3Alloc(m_capacity * sizeof(T));
			
			for (u32 i = 0; i < m_count; ++i)
			{
				T* old = oldElements + i;
				T* e = m_elements + i;
				new (e) T(*old);
				old->~T();
			}

			if (oldElements != m_localElements)
			{
				b3Free(oldElements);
			}
		}

		B3_ASSERT(m_count < m_capacity);
		T* e = m_elements + m_count;
		new(e) T(ele);
		++m_count;
	}

	void PopBack()
	{
		B3_ASSERT(m_count > 0);
		--m_count;
	}

	const T& Back() const
	{
		B3_ASSERT(m_count > 0);
		return m_elements[m_count - 1];
	}

	T& Back()
	{
		B3_ASSERT(m_count > 0);
		return m_elements[m_count - 1];
	}

	u32 Count() const
	{
		return m_count;
	}

	bool Empty() const
	{
		return m_count == 0;
	}

	void Resize(u32 size)
	{
		if (m_capacity < size)
		{
			// There is no capacity for the requested size.
			T* oldElements = m_elements;
			m_capacity = 2 * size;
			m_elements = (T*)b3Alloc(m_capacity * sizeof(T));

			// Construct and copy objects.
			for (u32 i = 0; i < m_count; ++i)
			{
				T* old = oldElements + i;
				T* e = m_elements + i;
				new (e) T(*old);
				old->~T();
			}

			// Construct objects up to the requested size.
			for (u32 i = m_count; i < size; ++i)
			{
				T* e = m_elements + i;
				new (e) T();
			}

			if (oldElements != m_localElements)
			{
				b3Free(oldElements);
			}

			m_count = size;
			return;
		}

		B3_ASSERT(m_capacity >= size);
		if (size < m_count)
		{
			// Destroy objects beyond the requested size.
			for (u32 i = size; i < m_count; ++i)
			{
				T* e = m_elements + i;
				e->~T();
			}
		}
		else
		{
			// Construct objects up to the requested size.
			for (u32 i = m_count; i < size; ++i)
			{
				T* e = m_elements + i;
				new (e) T();
			}
		}

		m_count = size;
	}

	void Swap(const b3ObjectArray<T>& other)
	{
		if (m_elements == other.m_elements)
		{
			return;
		}

		// Destroy all objects.
		for (u32 i = 0; i < m_count; ++i)
		{
			T* e = m_elements + i;
			e->~T();
		}

		// Ensure sufficient capacity for a copy.
		if (m_capacity < other.m_count)
		{
			if (m_elements != m_localElements)
			{
				b3Free(m_elements);
			}
			m_capacity = 2 * other.m_count;
			m_elements = (T*)b3Alloc(m_capacity * sizeof(T));
		}

		// Copy.
		B3_ASSERT(m_capacity >= other.m_count);
		for (u32 i = 0; i < other.m_count; ++i)
		{
			T* e2 = other.m_elements + i;
			T* e1 = m_elements + i;
			new (e1) T(*e2);
		}
		m_count = other.m_count;
	}
protected:
	b3ObjectArray(T* elements, u32 N)
	{
		B3_ASSERT(N > 0);
		m_localElements = elements;
		m_capacity = N;
		m_elements = m_localElements;
		m_count = 0;
	}

	b3ObjectArray(const b3ObjectArray<T>& other)
	{
		m_localElements = nullptr;
		m_capacity = 0;
		m_elements = nullptr;
		m_count = 0;

		Swap(other);
	}

	~b3ObjectArray()
	{
		if (m_elements != m_localElements)
		{
			for (u32 i = 0; i < m_count; ++i)
			{
				T* e = m_elements + i;
				e->~T();
			}
			b3Free(m_elements);
		}
	}

	void operator=(const b3ObjectArray<T>& other)
	{
		Swap(other);
	}

	u32 m_capacity;
	T* m_elements;
	u32 m_count;
	T* m_localElements;
};

template <typename T, u32 N>
class b3StackObjectArray : public b3ObjectArray<T>
{
public:
	b3StackObjectArray<T, N>() : b3ObjectArray<T>((T*)m_stackElements, N)
	{
	}

	b3StackObjectArray<T, N>(const b3StackObjectArray<T, N>& other) : b3ObjectArray<T>(other)
	{
	}

	b3StackObjectArray<T, N>(const b3ObjectArray<T>& other) : b3ObjectArray<T>(other)
	{
	}

	void operator=(const b3StackObjectArray<T, N>& other)
	{
		Swap(other);
	}

	void operator=(const b3ObjectArray<T>& other)
	{
		Swap(other);
	}

protected:
	u8 m_stackElements[N * sizeof(T)];
};

#endif
