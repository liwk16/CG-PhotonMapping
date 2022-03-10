#ifndef SCENE_H
#define SCENE_H

#include "raytracer.h"
#include "texture.h"

namespace RayTracer
{
#define HIT 1
#define MISS 0
#define INPRIM -1
	
	extern std::vector<Vector3> vertices;

	class Material
	{
	private:
		Color m_Color;
		Color m_Absorb;
		float m_Refl;
		float m_Diff;
		float m_Refr;
		float m_RefrIdx;
		float m_Spec;
		Texture m_Texture;
	public:
		int m_Tflag;
	
		Material();
		void SetColor(Color& a_Color) { m_Color = a_Color; m_Absorb = Color(1.0f, 1.0f, 1.0f) - a_Color; }
		Color GetColor() { 
			return m_Color; 
		}
		Color GetColor(int u, int v)
		{
			return m_Texture.getColor(u, v);
		}


		void SetDiffuse(float a_Diff) { m_Diff = a_Diff; }
		void SetReflection(float a_Refl) { m_Refl = a_Refl; }
		void SetSpecular(float a_Spec) { m_Spec = a_Spec; }
		void SetRefraction(float a_Refr) { m_Refr = a_Refr; }
		void SetRefractionIndex(float a_RefrIdx) { m_RefrIdx = a_RefrIdx; }
		void SetTexture(std::string pic_name);

		float GetSpecular() { return m_Spec; }
		float GetDiffuse() { return m_Diff; }
		float GetReflection() { return m_Refl; }
		float GetRefraction() { return m_Refr; }
		float GetRefractionIndex() { return m_RefrIdx; }
		Color GetAbsorb() { return m_Absorb; }
	};

	class Primitive
	{
	public:
		Material m_Material;
		char* m_Name;
		bool m_Light;
		Vector3 DX, DY;
	public:
		enum
		{
			SPHERE = 1,
			PLANE,
			REC,
			BEZIER,
			TRI
		};
		Primitive() :m_Name(0), m_Light(false) {};
		Material* GetMaterial() { return &m_Material; }
		void SetMaterial(Material& a_Mat) { m_Material = a_Mat; }
		virtual int GetType() = 0;
		virtual int Intersect(Ray& a_Ray, float& a_Dist) = 0;
		virtual Vector3 GetNormal(Vector3& a_Pos) = 0;
		virtual Color GetColor(Vector3& a_Pos) = 0; 
		virtual void Light(bool a_Light) { m_Light = a_Light; }

		virtual void SetDirection(Vector3& dx, Vector3& dy) { 
			NORMALIZE(dx);
			NORMALIZE(dy);
			DX = dx; DY = dy;
		}

		virtual Vector3 GetDX() { return DX; }
		virtual Vector3 GetDY() { return DY; }

		bool IsLight() { return m_Light; }
		void SetName(char* a_Name);
		char* GetName() { return m_Name; }
	};

	class Sphere : public Primitive
	{
	private:
		Vector3 m_Centre;
		float m_SqRadius, m_Radius, m_RRadius;

	public:
		Sphere(Vector3& a_Centre, float a_Radius) :m_Centre(a_Centre), m_Radius(a_Radius), m_SqRadius(a_Radius*a_Radius), m_RRadius(1.0f / a_Radius) {};
		int GetType() { return SPHERE; }
		Vector3& GetCentre() { return m_Centre; }
		float GetSqRadius() { return m_SqRadius; }
		int Intersect(Ray& a_Ray, float& a_Dist);
		Vector3 GetNormal(Vector3& a_Pos) { return (a_Pos - m_Centre)*m_RRadius; }
		Color GetColor(Vector3& a_Pos)
		{
			return m_Material.GetColor();
		}
	};

	class PlanePrim :public Primitive
	{
	private:
		Plane m_Plane;
		Vector3 m_Origin;
	public:
		int GetType() { return PLANE; }
		PlanePrim(Vector3& a_Normal, float a_D) :m_Plane(Plane(a_Normal, a_D)) {}
		PlanePrim(Vector3& a_Normal, Vector3& a_Origin) {
			NORMALIZE(a_Normal);
			m_Origin = a_Origin;
			float D = -(DOT(a_Normal, a_Origin));
			m_Plane = Plane(a_Normal, D);
		}
		Vector3& GetNormal() { return m_Plane.N; }
		float GetD() { return m_Plane.D; }
		int Intersect(Ray& a_Ray, float& a_Dist);
		Vector3 GetNormal(Vector3& a_Pos);
		
		Color GetColor(Vector3& a_Pos)
		{
			if (m_Material.m_Tflag == 0)return m_Material.GetColor();
			Vector3 rele_pos = a_Pos - m_Origin;
			float u = DOT(rele_pos, DX)*RESIZE;
			float v = DOT(rele_pos, DY)*RESIZE;
			return m_Material.GetColor(u, v);
		}
	};

	class Rectangle :public Primitive
	{
	private:
		Plane m_Plane;
		Vector3 m_Centre;
		float half_height, half_width;
	public:
		int GetType() { return REC; }

		Rectangle() {}
		Rectangle(Vector3& a_Normal, Vector3& a_Centre, float hhgt, float hwid)
		{
			NORMALIZE(a_Normal);
			m_Centre = a_Centre;
			half_height = hhgt;
			half_width = hwid;
			float D = -DOT(a_Normal, a_Centre);
			m_Plane = Plane(a_Normal, D);
		}
		Vector3& GetNormal() { return m_Plane.N; }
		Vector3 GetNormal(Vector3& a_Pos) { printf("Rec Normal\n"); return m_Plane.N; }

		int Intersect(Ray& a_Ray, float& a_Dist);

		Color GetColor(Vector3& a_Pos)
		{
			return m_Material.GetColor();
		}

		Vector3 GetCentre() { return m_Centre; }
		float GetHalfWidth() { return half_width; }
		float GetHalfHeight() { return half_height; }

		Vector3 GetRandPoint()
		{
			float x = Rand(2.0f) - 1.0f, y = Rand(2.0f) - 1.0f;
			Vector3 Point = m_Centre + x*half_width*DX + y*half_height*DY;
			return Point;
		}
	};

	class Triangle
	{
	public:
		int v0, v1, v2;
		Plane m_Plane;
		Eigen::Vector3f E1, E2;
		Vector3 m_Centre;
	public:
		static int layer;
		Triangle() {}
		Triangle(int a, int b, int c);
		int Intersect(Ray& a_Ray, float& a_Dist);
		int Judge(Vector3& a_Pos);
		void Init();
		Vector3 GetNormal()
		{
			return m_Plane.N;
		}

		bool operator < (const Triangle& other) const
		{
			if (layer == 0)
			{
				return this->m_Centre.x < other.m_Centre.x;
			}
			if (layer == 1)
			{
				return this->m_Centre.y < other.m_Centre.y;
			}
			return this->m_Centre.z < other.m_Centre.z;
		}
	};

	class aabb
	{
	private:
		Vector3 m_Pos, m_Size;
	public:
		aabb() :m_Pos(Vector3(0, 0, 0)), m_Size(Vector3(0, 0, 0)) {}
		aabb(Vector3& a_Pos, Vector3& a_Size) :m_Pos(a_Pos), m_Size(a_Size) {}
		void Set(Vector3& minPoint, Vector3& maxPoint,int type)
		{
			m_Pos = minPoint;
			m_Size = maxPoint - minPoint;
		}
		Vector3& GetPos() { return m_Pos; }
		Vector3& GetSize() { return m_Size; }

		int InterSect(Ray& a_Ray, float& a_Dist);	//不负责更新距离
		
		int InterSectForBezier(Ray& a_Ray, float& a_Dist, float& dist1, float& dist2);

		void Set(Vector3& a_Pos, Vector3& a_Size) { m_Pos = a_Pos; m_Size = a_Size; }
	};

	class Tri_Kdtree;

	class Tri_obj :public Primitive
	{
	public:
		//std::vector<Vector3> vertices;
		//std::vector<Vector3> Normal;
		//std::vector<int> TriangleNum;
		int Vertex_Count;
		int Face_Count;
		Vector3 m_Centre;//也就是offset，整个物体的平移量
		int Itst_Tri;
		Tri_Kdtree* tree;
	public:
		//void LoadModel(std::string path);//Fill vertices and triangles
		void build();//get Triangle prepared
		Tri_obj(Vector3& a_Centre, int ver, int face) :m_Centre(a_Centre), Vertex_Count(ver), Face_Count(face), Itst_Tri(-1) {}
		
		int GetType() { return TRI; }

		Vector3& GetNormal()
		{ 
			printf("break!\n");
			exit(0);
			return Vector3(0, 0, 0); 
		}
		Vector3 GetNormal(Vector3& a_Pos);
	
		Color GetColor(Vector3& a_Pos)
		{
			return m_Material.GetColor();
		}

		int Intersect(Ray& a_Ray, float& a_Dist);

	};




	class Scene
	{
	private:
		int m_Primitives;
		//int m_LightCount;
		Primitive** m_Primitive;
		//Primitive** m_Lights;
		std::vector<int> lightIndex;
	public:
		Scene() :m_Primitives(0), m_Primitive(0) { lightIndex.clear(); }
		~Scene();
		void InitScene();
		int GetNrPrimitives() { return m_Primitives; }
		//int GetLightCount() { return m_LightCount; }
		Primitive* GetPrimitive(int a_Idx) { return m_Primitive[a_Idx]; }
		void ReadObj(std::string path);
		//Primitive* GetLight(int a_Idx) { return m_Lights[a_Idx]; }
	};
}
#endif // !SCENE_H

