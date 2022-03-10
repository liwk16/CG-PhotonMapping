#ifndef RAYTRACER_H
#define RAYTRACER_H

#include "vector3.h"
#include <time.h>

namespace RayTracer {

	class Ray
	{
	private:
		Vector3 m_Origin;
		Vector3 m_Direction;
	public:
		Ray() :m_Origin(Vector3(0, 0, 0)), m_Direction(Vector3(0, 0, 0)) {};
		Ray(Vector3& a_Origin, Vector3& a_Dir);
		void SetOrigin(Vector3& a_Origin) { m_Origin = a_Origin; }
		void SetDirection(Vector3& a_Dir) { m_Direction = a_Dir; }
		Vector3& GetOrigin() { return m_Origin; }
		Vector3& GetDirection() { return m_Direction; }

	};

	class Scene;
	class Primitive;
	class Screen;
	class KdTree;
	class Photon;

	class Engine
	{
	protected:
		//float m_WX1, m_WY1, m_WX2, m_WY2, m_DX, m_DY, m_SX, m_SY;
		Scene* m_Scene;
		//Pixel* m_Dest;
		//int m_Width, m_Height, m_Currline, m_PPos;
		//Primitive** m_LastRow;
		Screen* m_Screen;
	public:
		KdTree* photonTree;
	public:
		Engine();
		~Engine();
		void SetTarget(Screen* screen);
		Scene* GetScene() { return m_Scene; }
		Primitive* Raytrace(Ray& a_Ray, Color& a_Acc, int a_Depth, float a_RIndex, float& a_Dist);
		Primitive* PhotonTrace(Photon ptn, int depth, float rIndex, float& distance);

		void PhotonShooting(Vector3 lightCentre, int PhotonCount);
		void buildTree();
		//void InitRender();
		void Render();

		void getCollide(Ray& ray, float& dis, int& state, Primitive*& prim);

	};
}


	


#endif
