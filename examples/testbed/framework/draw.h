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

#ifndef DRAW_H
#define DRAW_H

#include <bounce/bounce.h>

struct DrawPoints;
struct DrawLines;
struct DrawTriangles;
struct DrawWire;
struct DrawSolid;

//
class Camera
{
public:
	Camera();

	b3Mat44 BuildProjectionMatrix() const;
	b3Mat44 BuildViewMatrix() const;
	b3Transform BuildViewTransform() const;
	b3Mat44 BuildWorldMatrix() const;
	b3Transform BuildWorldTransform() const;
	
	b3Vec2 ConvertWorldToScreen(const b3Vec3& pw) const;
	b3Ray3 ConvertScreenToWorld(const b3Vec2& ps) const;

	float32 m_zoom;
	b3Vec3 m_center;
	b3Quat m_q;
	float32 m_width, m_height;
	float32 m_fovy;
	float32 m_zNear;
	float32 m_zFar;
};

class Draw : public b3Draw
{
public:
	Draw();
	~Draw();

	void SetViewMatrix(const b3Mat44& m);
	
	void SetProjectionMatrix(const b3Mat44& m);
	
	void EnableDrawPoints(bool flag);
	
	void EnableDrawLines(bool flag);
	
	void EnableDrawTriangles(bool flag);

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

	void DrawCapsule(const b3Vec3& p1, const b3Vec3& p2, float32 radius, const b3Color& color);

	void DrawSolidCapsule(const b3Vec3& p1, const b3Vec3& p2, float32 radius, const b3Color& color);

	void DrawPlane(const b3Vec3& normal, const b3Vec3& center, float32 radius, const b3Color& color);

	void DrawSolidPlane(const b3Vec3& normal, const b3Vec3& center, float32 radius, const b3Color& color);

	void DrawAABB(const b3AABB3& aabb, const b3Color& color);

	void DrawTransform(const b3Transform& xf);

	void DrawString(const b3Color& color, const b3Vec2& ps, const char* string, ...);
	
	void DrawString(const b3Color& color, const b3Vec3& pw, const char* string, ...);

	void DrawString(const b3Color& color, const char* string, ...);

	void DrawSolidSphere(const b3SphereShape* s, const b3Color& c, const b3Transform& xf);

	void DrawSolidCapsule(const b3CapsuleShape* s, const b3Color& c, const b3Transform& xf);
	
	void DrawSolidHull(const b3HullShape* s, const b3Color& c, const b3Transform& xf);
	
	void DrawSolidMesh(const b3MeshShape* s, const b3Color& c, const b3Transform& xf);

	void DrawSolidShape(const b3Shape* s, const b3Color& c, const b3Transform& xf);

	void DrawSolidShapes(const b3World& world);

	void Flush();
private:
	friend struct DrawPoints;
	friend struct DrawLines;
	friend struct DrawTriangles;

	DrawPoints* m_points;
	DrawLines* m_lines;
	DrawTriangles* m_triangles;

	friend struct DrawWire;
	friend struct DrawSolid;

	DrawWire* m_wire;
	DrawSolid* m_solid;
};

extern Camera* g_camera;
extern Draw* g_draw;

#endif