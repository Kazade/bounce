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

#ifndef PRESENTER_H
#define PRESENTER_H

#include <bounce/common/math/vec2.h>

class Model;
class View;

class Presenter
{
public:
	Presenter(Model* model, View* view);

	~Presenter();

	void Event_SetWindowSize(float w, float h);
	void Event_Press_Key(int button);
	void Event_Release_Key(int button);
	void Event_Press_Mouse(int button);
	void Event_Release_Mouse(int button);
	void Event_Move_Cursor(float x, float y);
	void Event_Scroll(float dx, float dy);
private:
	friend class Model;
	friend class View;

	Model* m_model;
	View* m_view;
};

#endif