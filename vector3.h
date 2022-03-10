#ifndef VECTOR3_H
#define VECTOR3_H

#include <math.h>
#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <string>
#include <math.h>
#include <fstream>

//using namespace Eigen;
/////
inline float Rand(float a_Range) { return ((float)rand() / RAND_MAX) * a_Range; }

namespace RayTracer {
	#define DOT(A,B) (A.x*B.x+A.y*B.y+A.z*B.z)
	#define NORMALIZE(A) {float l=1/sqrtf(A.x*A.x+A.y*A.y+A.z*A.z);A.x*=l;A.y*=l;A.z*=l;}
	#define LENGTH(A) (sqrtf(A.x*A.x+A.y*A.y+A.z*A.z))
	#define TRACEDEPTH 10
	#define PI 3.141592653589793238462f
	#define EPSILON	0.001f
	#define ABSORB_FACTOR 0.15f
	#define PHOTON 13000000
	#define RR 0.12f
	#define ZOOMFACTOR 900.0f
	#define RESIZE 10.0f
	//Ò»Ç§Íò 800.0f
	class Vector3
	{
	public:
		Vector3() :x(0.0f), y(0.0f), z(0.0f) {};
		Vector3(float a_x, float a_y, float a_z) :x(a_x), y(a_y), z(a_z) {};
		void Set(float a_x, float a_y, float a_z) { x = a_x; y = a_y; z = a_z; }
		//void Normalize();
		float Length() { return (float)sqrt(x*x + y*y + z*z); }
		float SqrLength() { return x*x + y*y + z*z; }
		float Dot(Vector3 a_V) { return x*a_V.x + y*a_V.y + z*a_V.z; }
		Vector3 Cross(Vector3 b) { return Vector3(y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x); }
		void operator += (Vector3& a_V) { x += a_V.x; y += a_V.y; z += a_V.z; }
		void operator += (Vector3* a_V) { x += a_V->x; y += a_V->y; z += a_V->z; }
		void operator -= (Vector3& a_V) { x -= a_V.x; y -= a_V.y; z -= a_V.z; }
		void operator -= (Vector3* a_V) { x -= a_V->x; y -= a_V->y; z -= a_V->z; }
		void operator *= (float f) { x *= f; y *= f; z *= f; }
		void operator *= (Vector3& a_V) { x *= a_V.x; y *= a_V.y; z *= a_V.z; }
		void operator *= (Vector3* a_V) { x *= a_V->x; y *= a_V->y; z *= a_V->z; }
		Vector3 operator- () const { return Vector3(-x, -y, -z); }
		friend Vector3 operator + (const Vector3& v1, const Vector3& v2) { return Vector3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z); }
		friend Vector3 operator - (const Vector3& v1, const Vector3& v2) { return Vector3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z); }
		friend Vector3 operator + (const Vector3& v1, Vector3* v2) { return Vector3(v1.x + v2->x, v1.y + v2->y, v1.z + v2->z); }
		friend Vector3 operator - (const Vector3& v1, Vector3* v2) { return Vector3(v1.x - v2->x, v1.y - v2->y, v1.z - v2->z); }
		friend Vector3 operator * (const Vector3& v, float f) { return Vector3(v.x * f, v.y * f, v.z * f); }
		friend Vector3 operator * (const Vector3& v1, Vector3& v2) { return Vector3(v1.x * v2.x, v1.y * v2.y, v1.z * v2.z); }
		friend Vector3 operator * (float f, const Vector3& v) { return Vector3(v.x * f, v.y * f, v.z * f); }

		union
		{
			struct { float x, y, z; };
			struct { float r, g, b; };
			struct { float cell[3]; };
		};
	};

	class Plane
	{
	public:
		Plane() :N(0, 0, 0), D(0) {};
		Plane(Vector3 a_Normal, float a_D) :N(a_Normal), D(a_D) {};
		union
		{
			struct
			{
				Vector3 N;
				Vector3 DX;
				Vector3 DY;
				float D;
			};
			float cell[10];
		};
	};

	typedef	Vector3 Color;
}
#endif // !VECTOR3_H

