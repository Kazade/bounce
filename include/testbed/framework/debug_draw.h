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

#ifndef DEBUG_DRAW_H
#define DEBUG_DRAW_H

#include <bounce/bounce.h>
#include "mat44.h"

struct DrawPoints;
struct DrawLines;
struct DrawTriangles;
struct DrawWire;
struct DrawSolid;

class Camera
{
public:
	Camera()
	{
		m_center.Set(0.0f, 5.0f, 0.0f);
		m_q.SetIdentity();
		m_width = 1024.0f;
		m_height = 768.0f;
		m_zNear = 1.0f;
		m_zFar = 500.0f;
		m_fovy = 0.25f * B3_PI;
		m_zoom = 10.0f;
	}

	Mat44 BuildProjectionMatrix() const;
	Mat44 BuildViewMatrix() const;
	b3Transform BuildViewTransform() const;
	Mat44 BuildWorldMatrix() const;
	b3Transform BuildWorldTransform() const;
	
	b3Vec2 ConvertWorldToScreen(const b3Vec3& pw) const;
	Ray3 ConvertScreenToWorld(const b3Vec2& ps) const;

	float32 m_zoom;
	b3Vec3 m_center;
	b3Quat m_q;
	float32 m_width, m_height;
	float32 m_fovy;
	float32 m_zNear;
	float32 m_zFar;
};

class DebugDraw : public b3Draw
{
public:
	DebugDraw();
	~DebugDraw();

	void DrawPoint(const b3Vec3& p, float32 size, const b3Color& color);

	void DrawSegment(const b3Vec3& p1, const b3Vec3& p2, const b3Color& color);
	
	void DrawTriangle(const b3Vec3& p1, const b3Vec3& p2, const b3Vec3& p3, const b3Color& color);

	void DrawSolidTriangle(const b3Vec3& normal, const b3Vec3& p1, const b3Vec3& p2, const b3Vec3& p3, const b3Color& color);

	void DrawPolygon(const b3Vec3* vertices, u32 count, const b3Color& color);
	
	void DrawSolidPolygon(const b3Vec3& normal, const b3Vec3* vertices, u32 count, const b3Color& color);

	void DrawCircle(const b3Vec3& normal, const b3Vec3& center, float32 radius, const b3Color& color);

	void DrawSolidCircle(const b3Vec3& normal, const b3Vec3& center, float32 radius, const b3Color& color);

	void DrawSphere(const b3Vec3& center, float32 radius, const b3Color& color);
	
	void DrawSolidSphere(const b3Vec3& center, float32 radius, const b3Color& color);

	void DrawAABB(const b3AABB3& aabb, const b3Color& color);

	void DrawTransform(const b3Transform& xf);

	//
	void DrawString(const char* string, const b3Color& color, ...);

	void DrawSphere(const b3SphereShape* s, const b3Color& c, const b3Transform& xf);

	void DrawCapsule(const b3CapsuleShape* s, const b3Color& c, const b3Transform& xf);
	
	void DrawHull(const b3HullShape* s, const b3Color& c, const b3Transform& xf);
	
	void DrawMesh(const b3MeshShape* s, const b3Color& c, const b3Transform& xf);

	void DrawShape(const b3Shape* s, const b3Color& c, const b3Transform& xf);

	void Draw(const b3World& world);

	void Draw();
private:
	friend struct DrawShapes;

	DrawPoints* m_points;
	DrawLines* m_lines;
	DrawTriangles* m_triangles;
	DrawWire* m_wire;
	DrawSolid* m_solid;
};

#endif