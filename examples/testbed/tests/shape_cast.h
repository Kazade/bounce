#ifndef SHAPE_CAST_H
#define SHAPE_CAST_H

class ShapeCast : public Test
{
public:
	ShapeCast()
	{
		m_shapeA.m_hull = &b3BoxHull_identity;
		m_shapeA.m_radius = 0.0f;

		m_shapeB.m_hull = &b3BoxHull_identity;
		m_shapeB.m_radius = 0.0f;

		m_xfA.position.Set(-5.0f, 0.0f, 0.0f);
		m_xfA.rotation.SetIdentity();

		m_xfB.position.Set(10.0f, 0.0f, 0.0f);
		m_xfB.rotation.SetIdentity();
		
		m_proxyA.Set(&m_shapeA, 0);
		m_proxyB.Set(&m_shapeB, 0);
	}

	void Step()
	{
		g_draw->DrawString(b3Color_white, "Left/Right/Up/Down Arrow - Translate shape");
		g_draw->DrawString(b3Color_white, "X/Y/Z - Rotate shape");

		g_draw->DrawTransform(m_xfA);
		g_draw->DrawTransform(m_xfB);

		m_world.DrawShape(m_xfA, &m_shapeA, b3Color_black);
		m_world.DrawShape(m_xfB, &m_shapeB, b3Color_black);

		m_world.DrawSolidShape(m_xfA, &m_shapeA, b3Color_white);
		m_world.DrawSolidShape(m_xfB, &m_shapeB, b3Color_white);

		b3Vec3 translationB = -100.0f * b3Vec3_x;
		g_draw->DrawSegment(m_xfB.position, m_xfB.position + translationB, b3Color_white);

		b3GJKShapeCastOutput out;
		bool hit = b3GJKShapeCast(&out, m_xfA, m_proxyA, m_xfB, m_proxyB, translationB);

		g_draw->DrawString(b3Color_white, "Iterations = %d", out.iterations);

		if (hit)
		{
			g_draw->DrawPoint(out.point, 4.0f, b3Color_green);
			g_draw->DrawSegment(out.point, out.point + out.normal, b3Color_green);

			b3Transform xfB;
			xfB.rotation = m_xfB.rotation;
			xfB.position = m_xfB.position + out.t * translationB;

			m_world.DrawShape(xfB, &m_shapeB, b3Color_black);
		}
	}

	void KeyDown(int key)
	{
		if (key == GLFW_KEY_LEFT)
		{
			m_xfB.position.x -= 0.105f;
		}

		if (key == GLFW_KEY_RIGHT)
		{
			m_xfB.position.x += 0.105f;
		}

		if (key == GLFW_KEY_UP)
		{
			m_xfB.position.y += 0.105f;
		}

		if (key == GLFW_KEY_DOWN)
		{
			m_xfB.position.y -= 0.105f;
		}

		if (key == GLFW_KEY_X)
		{
			b3Quat qx(b3Vec3(1.0f, 0.0f, 0.0f), 0.05f * B3_PI);
			b3Mat33 xfx = b3QuatMat33(qx);

			m_xfB.rotation = m_xfB.rotation * xfx;
		}

		if (key == GLFW_KEY_Y)
		{
			b3Quat qy(b3Vec3(0.0f, 1.0f, 0.0f), 0.05f * B3_PI);
			b3Mat33 xfy = b3QuatMat33(qy);

			m_xfB.rotation = m_xfB.rotation * xfy;
		}

		if (key == GLFW_KEY_Z)
		{
			b3Quat qy(b3Vec3(0.0f, 0.0f, 1.0f), 0.05f * B3_PI);
			b3Mat33 xfz = b3QuatMat33(qy);

			m_xfB.rotation = m_xfB.rotation * xfz;
		}
	}

	static Test* Create()
	{
		return new ShapeCast();
	}

	b3HullShape m_shapeA;
	b3Transform m_xfA;
	b3ShapeGJKProxy m_proxyA;

	b3HullShape m_shapeB;
	b3Transform m_xfB;
	b3ShapeGJKProxy m_proxyB;
};

#endif