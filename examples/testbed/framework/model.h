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

#ifndef MODEL_H
#define MODEL_H

#include <testbed/framework/debug_draw.h>
#include <testbed/framework/testbed_listener.h>
#include <testbed/framework/test.h>

class Model
{
public:
	Model();

	~Model();

	void Action_SaveTest();
	
	void Action_SelectTest(int selection);
	void Action_RestartTest();
	void Action_PreviousTest();
	void Action_NextTest();
	void Action_PlayPause();
	void Action_SingleStep();
	void Action_DefaultCamera();
	void Action_LeftCamera();
	void Action_RightCamera();
	void Action_BottomCamera();
	void Action_TopCamera();
	void Action_BackCamera();
	void Action_FrontCamera();

	void Command_Step();
	void Command_Press_Key(int button);
	void Command_Release_Key(int button);
	void Command_Press_Mouse_Left(const b3Vec2& ps);
	void Command_Release_Mouse_Left(const b3Vec2& ps);
	void Command_Move_Cursor(const b3Vec2& ps);
	void Command_ResizeCamera(float32 w, float32 h);
	void Command_RotateCameraX(float32 angle);
	void Command_RotateCameraY(float32 angle);
	void Command_TranslateCameraX(float32 d);
	void Command_TranslateCameraY(float32 d);
	void Command_ZoomCamera(float32 d);
private:
	friend class View;
	friend class Controller;

	DebugDraw m_debugDraw;
	Camera m_camera;
	Profiler m_profiler;
	TestbedListener m_profilerListener;

	Settings m_settings;
	Test* m_test;
};

inline void Model::Action_SelectTest(int selection)
{
	m_settings.testID = selection;
	m_settings.lastTestID = -1;
}

inline void Model::Action_RestartTest()
{
	m_settings.lastTestID = -1;
}

inline void Model::Action_PreviousTest()
{
	m_settings.testID = b3Clamp(m_settings.testID - 1, 0, int(g_testCount) - 1);
	m_settings.lastTestID = -1;
}

inline void Model::Action_NextTest()
{
	m_settings.testID = b3Clamp(m_settings.testID + 1, 0, int(g_testCount) - 1);
	m_settings.lastTestID = -1;
}

inline void Model::Action_SaveTest()
{
	m_test->Save();
}

inline void Model::Action_PlayPause()
{
	m_settings.pause = !m_settings.pause;
}

inline void Model::Action_SingleStep()
{
	m_settings.pause = true;
	m_settings.singleStep = true;
}

inline void Model::Action_DefaultCamera()
{
	m_camera.m_q = b3QuatRotationX(-0.125f * B3_PI);
	m_camera.m_center.SetZero();
	m_camera.m_zoom = 20.0f;
}

inline void Model::Action_LeftCamera()
{
	m_camera.m_q.Set(b3Vec3(0.0f, 1.0f, 0.0f), 0.5f * B3_PI);
	m_camera.m_center.SetZero();
	m_camera.m_zoom = 20.0f;
}

inline void Model::Action_RightCamera()
{
	m_camera.m_q.Set(b3Vec3(0.0f, 1.0f, 0.0f), -0.5f * B3_PI);
	m_camera.m_center.SetZero();
	m_camera.m_zoom = 20.0f;
}

inline void Model::Action_BottomCamera()
{
	m_camera.m_q.Set(b3Vec3(1.0f, 0.0f, 0.0f), 0.5f * B3_PI);
	m_camera.m_center.SetZero();
	m_camera.m_zoom = 20.0f;
}

inline void Model::Action_TopCamera()
{
	m_camera.m_q.Set(b3Vec3(1.0f, 0.0f, 0.0f), -0.5f * B3_PI);
	m_camera.m_center.SetZero();
	m_camera.m_zoom = 20.0f;
}

inline void Model::Action_BackCamera()
{
	m_camera.m_q.Set(b3Vec3(0.0f, 1.0f, 0.0f), -B3_PI);
	m_camera.m_center.SetZero();
	m_camera.m_zoom = 20.0f;
}

inline void Model::Action_FrontCamera()
{
	m_camera.m_q.SetIdentity();
	m_camera.m_center.SetZero();
	m_camera.m_zoom = 20.0f;
}

inline void Model::Command_Press_Key(int button)
{
	m_test->KeyDown(button);
}

inline void Model::Command_Release_Key(int button)
{
	m_test->KeyUp(button);
}

inline void Model::Command_Press_Mouse_Left(const b3Vec2& ps)
{
	Ray3 pw = m_camera.ConvertScreenToWorld(ps);

	m_test->MouseLeftDown(pw);
}

inline void Model::Command_Release_Mouse_Left(const b3Vec2& ps)
{
	Ray3 pw = m_camera.ConvertScreenToWorld(ps);

	m_test->MouseLeftUp(pw);
}

inline void Model::Command_Move_Cursor(const b3Vec2& ps)
{
	Ray3 pw = m_camera.ConvertScreenToWorld(ps);

	m_test->MouseMove(pw);
}

inline void Model::Command_ResizeCamera(float32 w, float32 h)
{
	m_camera.m_width = w;
	m_camera.m_height = h;
}

inline void Model::Command_RotateCameraX(float32 angle)
{
	b3Quat d = b3QuatRotationX(angle);

	m_camera.m_q = m_camera.m_q * d;
	m_camera.m_q.Normalize();
}

inline void Model::Command_RotateCameraY(float32 angle)
{
	b3Quat d = b3QuatRotationY(angle);

	m_camera.m_q = d * m_camera.m_q;
	m_camera.m_q.Normalize();
}

inline void Model::Command_TranslateCameraX(float32 d)
{
	b3Transform transform = m_camera.BuildWorldTransform();

	m_camera.m_center += d * transform.rotation.x;
}

inline void Model::Command_TranslateCameraY(float32 d)
{
	b3Transform transform = m_camera.BuildWorldTransform();

	m_camera.m_center += d * transform.rotation.y;
}

inline void Model::Command_ZoomCamera(float32 d)
{
	m_camera.m_zoom += d;
}

#endif