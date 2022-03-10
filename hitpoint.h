#ifndef HITPOINT_H
#define HITPOINT_H

#include "vector3.h"

namespace RayTracer
{
	class HitPoint
	{
	public:
		Vector3 pos;
		Vector3 normal;
		Vector3 ray_Dir;
		float BRDF;
		float x, y;
		float R;
		int photon_count;
		Color color;
	public:
		//static int layer;
		//bool operator <(const HitPoint& other) const
		//{
		//	if (layer = 0)
		//	{
		//		return this->pos.x < other.pos.x;
		//	}
		//	if (layer = 1)
		//	{
		//		return this->pos.y < other.pos.y;
		//	}
		//	return this->pos.z < other.pos.z;
		//}
	};


	class Photon
	{
	public:
		Vector3 ori;
		Vector3 dir;
		Color color;
		
		Photon() {}
		Photon(Vector3& or , Vector3& di, Color& col) :ori(or ), dir(di), color(col) {}
		static int layer;
		bool operator < (const Photon& other) const
		{
			if (layer == 0)
			{
				return this->ori.x < other.ori.x;
			}
			if (layer == 1)
			{
				return this->ori.y < other.ori.y;
			}
			return this->ori.z < other.ori.z;
		}
	};

	//std::vector<Photon> photonList;

}
#endif // !HITPOINT_H

