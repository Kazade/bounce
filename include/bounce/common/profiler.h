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

#ifndef B3_PROFILER_H
#define B3_PROFILER_H

#include <bounce/common/settings.h>

#define B3_PROFILE(name) b3ProfileScope scope(name)

// You should implement this function to use your own profiler.
bool b3PushProfileScope(const char* name);

// You should implement this function to use your own profiler.
void b3PopProfileScope();

struct b3ProfileScope
{
	b3ProfileScope(const char* name)
	{
		b = b3PushProfileScope(name);
	}

	~b3ProfileScope()
	{
		if (b)
		{
			b3PopProfileScope();
		}
	}
private:
	bool b;
};

#endif