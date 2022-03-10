#ifndef BEZIER_H
#define BEZIER_H

#include "vector3.h"
#include "scene.h"

namespace RayTracer
{
	extern int C[5][5];
	

	class Bezier :public Primitive
	{
		aabb box;
		std::vector<Eigen::Vector2f> m_ControlPoints;//二维控制点
		Vector3 m_Normal;
		Eigen::Matrix3f A;
	public:

		int GetType() { return BEZIER; }

		//需要由t得到P
		//
		Bezier()
		{
			float fac = 2.0f;
			//直接把控制点写在这好了,同时还有aabb的构造
			m_ControlPoints.clear();
			m_ControlPoints.push_back(convert(0.2f, 2.0f));
			m_ControlPoints.push_back(convert(0.1f, 1.2f));
			m_ControlPoints.push_back(convert(1.0f, 1.0f));
			m_ControlPoints.push_back(convert(0.3f, 0.0f));
			
			Vector3 offset{ 0.0f,0,0 };
			box.Set(fac*Vector3(-1, 0, -1) + offset, fac*Vector3(1, 2, 1) + offset, 0);
		}

		Eigen::Vector2f convert(float a, float b)
		{
			float fac = 2.0f;
			return Eigen::Vector2f(fac*a, fac*b);
		}

		Eigen::Vector2f P(float t)
		{
			Eigen::Vector2f p(0, 0);
			int Size = m_ControlPoints.size();
			int n = Size - 1;
			for (int i = 0; i < Size; ++i)
			{
				float Bt = float(C[n][i])*powf(t, i)*powf(1 - t, n - i);
				p += m_ControlPoints[i] * Bt;
			}
			return p;
		}

		Eigen::Vector3f S(float& t, float& cita)//表示面上的点
		{
			Eigen::Vector2f p = P(t);
			
			return Eigen::Vector3f(p[0] * cosf(cita), p[1], p[0] * sinf(cita));
		}

		Eigen::Vector2f dPdt(float& t)
		{
			Eigen::Vector2f p(0, 0);
			int Size = m_ControlPoints.size();
			int n = Size - 1;
			for (int i = 0; i < Size; ++i)
			{
				float fac = float(i) - float(n)*t;
				float dBt = C[n][i] * fac*powf(t, i - 1)*powf(1 - t, n - i - 1);
				p += m_ControlPoints[i] * dBt;
			}
			return p;
		}

		Eigen::Vector3f dSdt(float& t, float& cita)
		{
			Eigen::Vector2f p = dPdt(t);
			return Eigen::Vector3f(cosf(cita)*p[0], p[1], sinf(cita)*p[0]);
		}

		Eigen::Vector3f dSdc(float& t, float& cita)
		{
			Eigen::Vector2f p = P(t);
			return Eigen::Vector3f(-sinf(cita)*p[0], 0, cosf(cita)*p[0]);
		}

		Eigen::Matrix3f& dF(Eigen::Vector3f& dir, Eigen::Vector3f& dSdt, Eigen::Vector3f& dSdc)
		{
			//Eigen::Matrix3f A;
			A << dir, -dSdt, -dSdc;
			return A;
		}

		Eigen::Vector3f F(Ray& a_Ray, float& dis, float& t, float& cita)
		{
			Vector3 L = a_Ray.GetOrigin() + a_Ray.GetDirection()*dis;
			Eigen::Vector3f p = S(t, cita);
			return Eigen::Vector3f(L.x, L.y, L.z) - p;
		}

		int Intersect(Ray& a_Ray, float& a_Dist)
		{
			float dist0 = a_Dist;
			float dist1 = 0.0f, dist2 = 0.0f;
			if (box.InterSectForBezier(a_Ray, dist0, dist1, dist2))
			{
				//与盒子有交点
				//dist1 ,dist2 给出了t的范围
				if (dist1 > dist2)
				{
					float temp = dist1;
					dist1 = dist2;
					dist2 = temp;
				}
				//开始牛顿迭代，要求同时得出焦点的法向

				float finaldis = 100000.0f, finalt, finalcita;
				int flag = 0;

				for (int k = 0; k < 15; ++k)
				{
					float dis = Rand(dist2 - dist1) + dist1;
					if (k == 0)dis = dist1;
					if (k == 9)dis = dist2;
					//float dis = dist1;
					if (dis < 0)continue;
					float t = Rand(1.0f);
					float cita = Rand(1.0f) * 2 * PI;
					Eigen::Vector3f xi(dis, t, cita);//至此 产生一个初值

					for (int m = 0; m < 20; ++m)//迭代20轮
					{
						Eigen::Vector3f xi1 = Newton(a_Ray, xi[0], xi[1], xi[2]);//一个新解
						if (xi == xi1)break;
						xi = xi1;
					}

					//得到一个初值对应的最优结果xi
					if (xi[0] < EPSILON || Module(F(a_Ray, xi[0], xi[1], xi[2])) >= 0.01f || xi[1] < 0 || xi[1]>1 || cita < 0 || cita>2 * PI)continue;
					if (xi[0] < finaldis)
					{
						finaldis = xi[0];
						finalt = xi[1];
						finalcita = xi[2];
						flag = 1;
					}
					
					//final 为最终的三个参数
				}
				if (flag) {
					a_Dist = finaldis;

					//更新法向量
					Eigen::Vector2f dpdt = dPdt(finalt);
					Eigen::Vector2f Rt(dpdt[1], -dpdt[0]);
					m_Normal = { cosf(finalcita)*dpdt[1],-dpdt[0],sinf(finalcita)*dpdt[1] };
					NORMALIZE(m_Normal);
					Vector3 lightdir = a_Ray.GetDirection();
					float dot = DOT(lightdir, m_Normal);
					if (dot > 0)m_Normal = -m_Normal;

					return HIT;
				}
			}
			return MISS;
		}

		float Module(Eigen::Vector3f f)
		{
			return sqrtf(f.dot(f));
		}

		Eigen::Vector3f Newton(Ray& a_Ray,float& dist, float& t, float& cita)
		//Eigen::Vector3f Newton(Ray& a_Ray, Eigen::Vector3f& xi)
		{
			Eigen::Vector3f xi(dist, t, cita);
			Vector3 dir3 = a_Ray.GetDirection();
			Eigen::Vector3f dir(dir3.x, dir3.y, dir3.z);
			//Eigen::Matrix3f dfin = dF(dir, dSdt(t, cita), dSdc(t, cita)).inverse();
			//Eigen::Vector3f f = F(a_Ray, dist, t, cita);
			Eigen::Vector3f xi1 = xi - dF(dir, dSdt(t, cita), dSdc(t, cita)).inverse()*F(a_Ray,dist,t,cita);
			return xi1;
		}

		Vector3 GetNormal(Vector3& a_Pos) { return m_Normal; }

		Color GetColor(Vector3& a_Pos)
		{
			return m_Material.GetColor();
		}
	};
}


#endif // !BEZIER_H
