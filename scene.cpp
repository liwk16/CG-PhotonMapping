#include "vector3.h"
#include "scene.h"
#include <string.h>
#include "tri_kd.h"
#include "bezier.h"

namespace RayTracer {

	std::vector<Vector3> vertices;
	std::vector<Triangle> triangles;

	Triangle::Triangle(int a, int b, int c)
	{
		v0 = a; v1 = b; v2 = c;
	}

	void Triangle::Init()
	{
		if (v2 >= vertices.size()) 
		{
			printf("%d\n", v2); exit(0); 
		}
		Vector3 e1 = vertices[v0] - vertices[v1];
		Vector3 e2 = vertices[v0] - vertices[v2];
		E1 = { e1.x,e1.y,e1.z };
		E2 = { e2.x,e2.y,e2.z };

		float fac = 1.0f / 3.0f;
		m_Centre = (vertices[v0] + vertices[v1] + vertices[v2])*fac;

		Vector3 a_Normal = e2.Cross(e1);//法向不对就改这
		NORMALIZE(a_Normal);
		float D = -DOT(a_Normal, vertices[v0]);
		m_Plane = Plane(a_Normal, D);
	}

	int Triangle::Intersect(Ray& a_Ray, float& a_Dist)
	{
		float step = DOT(m_Plane.N, a_Ray.GetDirection());
		if (step != 0)
		{
			float dist = -(DOT(m_Plane.N, a_Ray.GetOrigin()) + m_Plane.D) / step;
			if (dist > 0 && dist < a_Dist)
			{
				Vector3 hitpoint = a_Ray.GetOrigin() + a_Ray.GetDirection()*dist;
				if (Judge(hitpoint))
				{
					a_Dist = dist;
					return (DOT(m_Plane.N, a_Ray.GetDirection()) > 0) ? INPRIM : HIT;
				}
				
			}
		}
		return MISS;
	}

	Vector3 Tri_obj::GetNormal(Vector3& a_Pos)
	{
		return triangles[Itst_Tri].GetNormal();
	}

	int Tri_obj::Intersect(Ray& a_Ray, float& a_Dist)
	{
		int state = MISS;
		this->tree->GetTriList(a_Ray, a_Dist);
		int count = this->tree->search_rlt.size();
		for (int i = 0; i < count; ++i)
		{
			int res;
			if (res = triangles[this->tree->search_rlt[i]].Intersect(a_Ray, a_Dist))
			{
				Itst_Tri = this->tree->search_rlt[i];
				state = res;
			}
		}

		return state;
		//后加和box
		//int state = MISS;
		//int Tri_Count = triangles.size();
		//for (int i = 0; i < Tri_Count; ++i)
		//{
		//	//Primitive* pr = m_Scene->GetPrimitive(i);
		//	int res;
		//	if (res = triangles[i].Intersect(a_Ray, a_Dist))
		//	{
		//		Itst_Tri = i;
		//		state = res;
		//	
		//	}
		//}
		////if (state == INPRIM) { printf("cuola\n"); }
		//return state;
	}

	void Tri_obj::build()
	{
		for (int i = 0; i < triangles.size(); ++i)
		{
			triangles[i].Init();
		}
	}

	int Triangle::Judge(Vector3& a_Pos)
	{
		Eigen::MatrixXf A(3,2);
		A << E1, E2;
		Vector3 S = vertices[v0] - a_Pos;
		Eigen::Vector3f SS{ S.x,S.y,S.z };
		Eigen::Vector2f ans = A.colPivHouseholderQr().solve(SS);

		if (ans[0] < 0) { return 0; }
		if (ans[1] < 0) { return 0; }
		if (ans[0] + ans[1] > 1.0f) { return 0; }
		if (isnan(ans[0]) || isinf(ans[0]))
		{
			printf("Warning: Triangle InterSect : Location not a number!\n");
			return 0;
		}
		return 1;
	}

	int aabb::InterSect(Ray& a_Ray, float& a_Dist)//不改变dist，只判断是否小于，aabb相交只用来判定要加入哪些面片求交
	{
		float dist[6];
		Vector3 ip[6], dir = a_Ray.GetDirection(), ori = a_Ray.GetOrigin();
		//int ret = MISS;
		for (int i = 0; i < 6; ++i)
		{
			dist[i] = -1;
		}
		Vector3 v1 = m_Pos, v2 = m_Pos + m_Size;
		if (dir.x)
		{
			float rc = 1.0f / dir.x;
			dist[0] = (v1.x - ori.x)*rc;
			dist[3] = (v2.x - ori.x)*rc;
		}
		if (dir.y)
		{
			float rc = 1.0f / dir.y;
			dist[1] = (v1.y - ori.y)*rc;
			dist[4] = (v2.y - ori.y)*rc;
		}
		if (dir.z)
		{
			float rc = 1.0f / dir.z;
			dist[2] = (v1.z - ori.z)*rc;
			dist[5] = (v2.z - ori.z)*rc;
		}
		for (int i = 0; i < 6; ++i)
		{
			if (dist[i] > 0 && dist[i] < a_Dist)
			{
				ip[i] = ori + dist[i] * dir;
				if ((ip[i].x > (v1.x - EPSILON)) &&
					(ip[i].x < (v2.x + EPSILON)) &&
					(ip[i].y > (v1.y - EPSILON)) &&
					(ip[i].y < (v2.y + EPSILON)) &&
					(ip[i].z > (v1.z - EPSILON)) &&
					(ip[i].z < (v2.z + EPSILON)))
				{
					return HIT;
				}
			}
		}
		return MISS;
	}

	int aabb::InterSectForBezier(Ray& a_Ray, float& a_Dist, float& dist1, float& dist2)//会改动距离,所以传进来就传一个复制的dist
	{
		float dist[6];
		
		Vector3 ip[6], dir = a_Ray.GetDirection(), ori = a_Ray.GetOrigin();
		int ret = MISS;
		for (int i = 0; i < 6; ++i)
		{
			dist[i] = -1;
		}
		Vector3 v1 = m_Pos, v2 = m_Pos + m_Size;
		if (dir.x)
		{
			float rc = 1.0f / dir.x;
			dist[0] = (v1.x - ori.x)*rc;
			dist[3] = (v2.x - ori.x)*rc;
		}
		if (dir.y)
		{
			float rc = 1.0f / dir.y;
			dist[1] = (v1.y - ori.y)*rc;
			dist[4] = (v2.y - ori.y)*rc;
		}
		if (dir.z)
		{
			float rc = 1.0f / dir.z;
			dist[2] = (v1.z - ori.z)*rc;
			dist[5] = (v2.z - ori.z)*rc;
		}

		int r = -1;

		for (int i = 0; i < 6; ++i)
		{
			if (dist[i] > 0 && dist[i] < a_Dist)
			{
				ip[i] = ori + dist[i] * dir;
				if ((ip[i].x > (v1.x - EPSILON)) &&
					(ip[i].x < (v2.x + EPSILON)) &&
					(ip[i].y >(v1.y - EPSILON)) &&
					(ip[i].y < (v2.y + EPSILON)) &&
					(ip[i].z >(v1.z - EPSILON)) &&
					(ip[i].z < (v2.z + EPSILON)))
				{
					a_Dist = dist[i];
					ret = HIT;
					r = i;
					dist1 = dist[i];
					dist2 = dist[(i + 3) % 6];
					//return HIT;
				}
			}
		}
		return ret;
	}

	Material::Material() :m_Color(Color(0.2f, 0.2f, 0.2f)), m_Refl(0), m_Diff(0.0f),m_Tflag(0) {}

	void Material::SetTexture(std::string pic_name)
	{
		m_Texture.setImage(pic_name);
		m_Tflag = 1;
	}


	void Primitive::SetName(char* a_Name) {
		delete m_Name;
		m_Name = new char[strlen(a_Name) + 1];
		strcpy(m_Name, a_Name);
	}

	int Sphere::Intersect(Ray& a_Ray, float& a_Dist)
	{
		Vector3 v = m_Centre - a_Ray.GetOrigin();
		float cast = DOT(v, a_Ray.GetDirection());
		float det = (cast*cast) + m_SqRadius - DOT(v, v);
		int rlt = MISS;
		if (det > 0)
		{
			det = sqrtf(det);
			float d1 = cast - det;
			float d2 = cast + det;
			if (d2 > 0)
			{
				if (d1 < 0)
				{
					if (d2 < a_Dist)
					{
						a_Dist = d2;
						rlt = INPRIM;
					}
				}
				else
				{
					if (d1 < a_Dist)
					{
						a_Dist = d1;
						rlt = HIT;
					}
				}
			}
		}
		return rlt;
	}

	int PlanePrim::Intersect(Ray& a_Ray, float& a_Dist)
	{
		float step = DOT(m_Plane.N, a_Ray.GetDirection());
		if (step != 0)
		{
			float dist = -(DOT(m_Plane.N, a_Ray.GetOrigin()) + m_Plane.D) / step;
			if (dist > 0)
			{
				if (dist < a_Dist)
				{
					a_Dist = dist;
					return HIT;
				}
			}
		}
		return MISS;
	}

	Vector3 PlanePrim::GetNormal(Vector3& a_Pos)
	{
		return m_Plane.N;	
	}

	int Rectangle::Intersect(Ray& a_Ray, float& a_Dist)
	{
		float step = DOT(m_Plane.N, a_Ray.GetDirection());
		if (step != 0)
		{
			float dist = -(DOT(m_Plane.N, a_Ray.GetOrigin()) + m_Plane.D) / step;
			if (dist > 0 && dist < a_Dist)
			{
				Vector3 hitpoint = a_Ray.GetOrigin() + a_Ray.GetDirection()*dist;
				Vector3 rele_vec = hitpoint - m_Centre;
				float x = DOT(rele_vec, DX);
				//float y = DOT(rele_vec, DY);
	
				if (abs(x)<half_width)
				{
					float y = DOT(rele_vec, DY);
					if (abs(y) < half_height)
					{
						//printf("hit light!\n");
						a_Dist = dist;
						return HIT;
					}
				}
			}
		}
		return MISS;
	}

	Scene::~Scene()
	{
		delete m_Primitive;
	}


	void Scene::ReadObj(std::string path)
	{
		vertices.clear();
		triangles.clear();
		std::ifstream fin(path);
		char type;
		float x, y, z;
		int v0, v1, v2;
		int ver, face;
		fin >> type;
		fin >> ver;
		fin >> face;
		face += ver;

		vertices.push_back(Vector3(0, 0, 0));
		//printf("vertex: ")
		while (face--)
		{
			fin >> type;
			if (type == 'v')
			{
				fin >> x >> y >> z;
				Vector3 vertex(-x, y, z);
				float fac = 2.0f;
				vertex = vertex*fac + Vector3(-4.0f, 1.8f, -4.0f);
				vertices.push_back(vertex);
				//printf("%g,%g,%g\n", x, y, z);
			}
			else
			{
				fin >> v0 >> v1 >> v2;
				Triangle tri(v0, v1, v2);
				triangles.push_back(tri);
				//printf("%d,%d,%d\n", v0, v1, v2);
			}
		}

		fin.close();
		printf("vertices : %d\nTriangles : %d\n", vertices.size(), triangles.size());
		printf("Model Loading Complete\n");

	}

	void Scene::InitScene()
	{
		m_Primitive = new Primitive*[500];
		//m_Lights = new Primitive*[10];
		//m_Lights[0] = new Rectangle(Vector3(1,-1,0),Vector3(-5,7,-4),0.3f,0.3f);
		//m_Lights[0]->Light(true);
		//m_Lights[0]->GetMaterial()->SetColor(Color(1.0f, 1.0f, 1.0f));
		//m_Lights[0]->SetDirection(Vector3(0, 0, 1), Vector3(1, 1, 0));
		//
		//m_LightCount = 1;
		//printf("Scene init done\n");

		//m_Primitive[0] = new Rectangle(Vector3(1,-0.6f,0),Vector3(-6,6,0),2.0f,2.0f);

		m_Primitive[0] = new Rectangle(Vector3(-1,-1,1), Vector3(6, 6, -6), 1.3f, 1.3f);
		m_Primitive[0]->Light(true);
		m_Primitive[0]->GetMaterial()->SetColor(Color(1.0f, 1.0f, 1.0f));
		m_Primitive[0]->SetDirection(Vector3(1, 0, 1), Vector3(-1, 1, 2));
		
		
		//// ground plane

		//m_Primitive[0] = new Sphere(Vector3(6, 6, -6), 0.1f);
		//m_Primitive[0]->Light(true);
		//m_Primitive[0]->GetMaterial()->SetColor(Color(1.0f, 1.0f, 1.0f));
		//

		//m_Primitive[1] = new PlanePrim(Vector3(0, 1, 0), 0.0f);
		m_Primitive[1] = new PlanePrim(Vector3(0, 1, 0), Vector3(-12, 0, -8));
		m_Primitive[1]->SetName("plane");
		m_Primitive[1]->GetMaterial()->SetReflection(0.3f);
		m_Primitive[1]->GetMaterial()->SetRefraction(0.0f);
		m_Primitive[1]->GetMaterial()->SetDiffuse(0.7f);
		m_Primitive[1]->GetMaterial()->SetSpecular(0.0f);
		m_Primitive[1]->GetMaterial()->SetColor(Color(1.0f, 1.0f, 1.0f));
		m_Primitive[1]->GetMaterial()->SetTexture("good.jpg");
		m_Primitive[1]->SetDirection(Vector3(1, 0, 0), Vector3(0, 0, 1));
		//m_Primitive[1]->GetMaterial()->SetColor(Color(1.0f, 1.0f, 1.0f));


		////blue
		//
		m_Primitive[2] = new Sphere(Vector3(2.0f, 0.5f, -3.0f), 0.5f);
		m_Primitive[2]->SetName("small sphere");
		m_Primitive[2]->GetMaterial()->SetDiffuse(0.0f);
		m_Primitive[2]->GetMaterial()->SetReflection(0.0f);
		m_Primitive[2]->GetMaterial()->SetRefraction(1.0f);
		m_Primitive[2]->GetMaterial()->SetSpecular(0.0f);
		m_Primitive[2]->GetMaterial()->SetRefractionIndex(1.5f);
		m_Primitive[2]->GetMaterial()->SetColor(Color(0.0f, 0.0f, 1.0f));
		//


		m_Primitive[3] = new PlanePrim(Vector3(0.4f, 0, -1), Vector3(-9, 0, 0));
		m_Primitive[3]->SetName("plane");
		m_Primitive[3]->GetMaterial()->SetReflection(0.7f);
		m_Primitive[3]->GetMaterial()->SetRefraction(0.0f);
		m_Primitive[3]->GetMaterial()->SetDiffuse(0.3f);
		m_Primitive[3]->GetMaterial()->SetSpecular(0.0f);
		m_Primitive[3]->GetMaterial()->SetColor(Color(1.0f, 1.0f, 1.0f));
		m_Primitive[3]->GetMaterial()->SetTexture("good.jpg");
		m_Primitive[3]->SetDirection(Vector3(1, 0, 0), Vector3(0, 0, 1));


		//// pink

		m_Primitive[4] = new Sphere(Vector3(0.3f, 0.5f, -4.0f), 0.5f);
		m_Primitive[4]->SetName("Sphere 3");
		m_Primitive[4]->GetMaterial()->SetDiffuse(0.0f);
		m_Primitive[4]->GetMaterial()->SetReflection(0.0f);
		m_Primitive[4]->GetMaterial()->SetRefraction(1.0f);
		m_Primitive[4]->GetMaterial()->SetSpecular(0.0f);
		m_Primitive[4]->GetMaterial()->SetRefractionIndex(1.5f);
		m_Primitive[4]->GetMaterial()->SetColor(Color(1.0f, 0.0f, 1.0f));
		
		// yellow
		m_Primitive[5] = new Sphere(Vector3(4.3f, 0.5f, -0.5f), 0.5f);
		m_Primitive[5]->SetName("qiu");
		m_Primitive[5]->GetMaterial()->SetDiffuse(0.0f);
		m_Primitive[5]->GetMaterial()->SetReflection(0.0f);
		m_Primitive[5]->GetMaterial()->SetRefraction(1.0f);
		m_Primitive[5]->GetMaterial()->SetSpecular(0.0f);
		m_Primitive[5]->GetMaterial()->SetRefractionIndex(1.5f);
		m_Primitive[5]->GetMaterial()->SetColor(Color(1.0f, 1.0f, 0));
		
		//refl
		m_Primitive[6] = new Sphere(Vector3(4.0f, 1.0f, 3.0f), 1.0f);
		m_Primitive[6]->SetName("ha");
		m_Primitive[6]->GetMaterial()->SetDiffuse(0.3f);
		m_Primitive[6]->GetMaterial()->SetReflection(0.7f);
		m_Primitive[6]->GetMaterial()->SetRefraction(0.0f);
		m_Primitive[6]->GetMaterial()->SetSpecular(0.0f);
		m_Primitive[6]->GetMaterial()->SetRefractionIndex(0.0f);
		m_Primitive[6]->GetMaterial()->SetColor(Color(1.0f, 1.0f, 1.0f));



		m_Primitive[7] = new Bezier();
		m_Primitive[7]->SetName("b");
		m_Primitive[7]->GetMaterial()->SetRefraction(0.0f);
		m_Primitive[7]->GetMaterial()->SetDiffuse(0.6f);
		m_Primitive[7]->GetMaterial()->SetReflection(0.4f);
		m_Primitive[7]->GetMaterial()->SetSpecular(0.0f);
		m_Primitive[7]->GetMaterial()->SetColor(Color(0.53f, 0.81f, 0.92f));


		m_Primitive[8] = new Tri_obj(Vector3(0, 0, 0), 0, 0);
		m_Primitive[8]->SetName("TRi");
		m_Primitive[8]->GetMaterial()->SetReflection(0.0f);//0.1
		m_Primitive[8]->GetMaterial()->SetRefraction(0.0f);//0.4
		m_Primitive[8]->GetMaterial()->SetDiffuse(1.0f);
		m_Primitive[8]->GetMaterial()->SetSpecular(0.0f);
		m_Primitive[8]->GetMaterial()->SetRefractionIndex(1.5f);
		m_Primitive[8]->GetMaterial()->SetColor(Color(0.0f, 1.0f, 0.0f));
		((Tri_obj*)m_Primitive[8])->build();
		((Tri_obj*)m_Primitive[8])->tree = new Tri_Kdtree();
		((Tri_obj*)m_Primitive[8])->tree->Init();

		int prim = 9;
		
		m_Primitives = prim;
		printf("Init Scene Done!\n");
	}
}