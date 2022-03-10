#include "raytracer.h"
#include "scene.h"
#include "screen.h"
#include "kdtree.h"
#include "hitpoint.h"

namespace RayTracer
{
	extern std::vector<Photon> photonList;

	Ray::Ray(Vector3& a_Origin, Vector3& a_Dir) :m_Origin(a_Origin), m_Direction(a_Dir) {}

	Engine::Engine()
	{
		m_Scene = new Scene();
		photonTree = new KdTree();
		srand(time(NULL));
	}

	Engine::~Engine()
	{
		delete m_Scene;
	}

	void Engine::SetTarget(Screen* a_screen)
	{
		m_Screen = a_screen;
	}

	void Engine::getCollide(Ray& ray, float& dis, int& state, Primitive*& prim)
	{
		for (int i = 0; i < m_Scene->GetNrPrimitives(); ++i)
		{
			Primitive* pr = m_Scene->GetPrimitive(i);
			int res;
			if (res = pr->Intersect(ray, dis))
			{
				prim = pr;
				state = res;
			}
		}
	}

	Primitive* Engine::PhotonTrace(Photon ptn, int depth, float rIndex, float& distance)
	{
		if (depth > TRACEDEPTH)return 0;
		distance = 1000000.0f;
		//printf("photonList size : %d\n", photonList.size());
		Vector3 intersectionPoint;
		Primitive* prim = 0;
		int rlt;

		for (int i = 0; i < m_Scene->GetNrPrimitives(); ++i)
		{
			Primitive* pr = m_Scene->GetPrimitive(i);
			int res;
			if (res = pr->Intersect(Ray(ptn.ori, ptn.dir),distance))
			{
				prim = pr;
				rlt = res;
			}
		}

		if (!prim)
		{
			//printf("没碰到\n");
			return 0;
		}
		if (prim->IsLight()) { //printf("碰到光源");
		return 0; }
		else
		{
			//printf("touch ");
			//printf("%s\n", prim->GetName());
			intersectionPoint = ptn.ori + distance*ptn.dir;

			float choice = Rand(1.0f);
			choice -= prim->GetMaterial()->GetDiffuse();
			if (choice < 0)
			{
				//漫反射
				//printf("diffusing\n");
				//if(rlt>0)photonList.push_back(Photon(intersectionPoint, ptn.dir, ptn.color));//edit
				photonList.push_back(Photon(intersectionPoint, ptn.dir, ptn.color));
				//return 0;

				float x = Rand(1.0f)-0.5f, y = Rand(1.0f)-0.5f, z = Rand(1.0f)-0.5f;
				Vector3 N = prim->GetNormal(intersectionPoint);
				Vector3 L = ptn.dir;
				Vector3 R = L - 2.0f*DOT(L, N)*N;

				Vector3 direction(x, y, z);
				NORMALIZE(direction);
				Vector3 diff_dir = R + 0.2f*direction;
				NORMALIZE(diff_dir);
				PhotonTrace(Photon(intersectionPoint+direction*EPSILON, diff_dir, ptn.color*prim->GetColor(intersectionPoint)),
					depth + 1, rIndex, distance);
				return prim;

			}
			choice -= prim->GetMaterial()->GetReflection();
			if (choice < 0)
			{
				//反射
				//printf("reflecting\n");
				Vector3 N = prim->GetNormal(intersectionPoint);
				Vector3 L = ptn.dir;
				Vector3 R = L - 2.0f*DOT(L, N)*N;
				NORMALIZE(R);
				PhotonTrace(Photon(intersectionPoint+R*EPSILON, R, ptn.color*prim->GetColor(intersectionPoint)),
					depth + 1, rIndex, distance);
			}
			else
			{
				//折射
				//printf("refracting\n");
				float rIdx = prim->GetMaterial()->GetRefractionIndex();
				float n = rIndex / rIdx;
				Vector3 I = ptn.dir;
				Vector3 N = prim->GetNormal(intersectionPoint)*(float)rlt;
				float cosI = -DOT(I, N);
				float cosT2 = 1.0f - (1.0f - cosI*cosI)*n*n;
				if (cosT2 > 0.0f)
				{
					Vector3 T = n*I + (n*cosI - sqrtf(cosT2))*N;
					NORMALIZE(T);
					Color col = ptn.color;
					float dis = 0.0f;
					if (rlt > 0)col = col*prim->GetColor(intersectionPoint);
					PhotonTrace(Photon(intersectionPoint + T*EPSILON, T, col),
						depth + 1, rIdx, dis);
				}
			}
			
		}
		return prim;
	}

	void Engine::PhotonShooting(Vector3 lightCentre, int PhotonCount)
	{

		//while (PhotonCount--)
		//{
		//	float x = Rand(2.0f) - 1.0f, y = Rand(2.0f) - 1.0f, z = Rand(2.0f) - 1.0f;
		//	Vector3 dir(x, y, z);
		//	NORMALIZE(dir);
		//	float dis = 0.0f;
		//	PhotonTrace(Photon(lightCentre, dir, Color(1.0f, 1.0f, 1.0f)),
		//		1, 1.0f, dis);
		//	//std::cout << PhotonCount << std::endl;
		//}
		//return;


		Primitive* prim = m_Scene->GetPrimitive(0);
		Vector3 N = prim->GetNormal(Vector3(0,0,0));
		
		int sample = 100;
		int ptnCount = PhotonCount / sample;
		int tempCount = ptnCount;
		
		for (int i = 0; i < 100; ++i)
		{
			Vector3 lightPoint = ((Rectangle*)prim)->GetRandPoint();
			//printf("%g,%g,%g\n", lightPoint.x, lightPoint.y, lightPoint.z);
			while (tempCount--)
			{
				float x = Rand(2.0f) - 1.0f, y = Rand(2.0f) - 1.0f, z = Rand(2.0f) - 1.0f;
				Vector3 dir(x, y, z);
				float dot = DOT(dir, N);
				if (dot <= 0)dir = -dir;
				NORMALIZE(dir);
				float dis = 0.0f;
				PhotonTrace(Photon(lightPoint + dir*EPSILON, dir, Color(1.0f, 1.0f, 1.0f)), 
						1, 1.0f, dis);
			}
			tempCount = ptnCount;
			printf("light point %d finished\r", i);
		}

		printf("\n");
		//while (PhotonCount--)
		//{
		//	float x = Rand(2.0f) - 1.0f, y = Rand(2.0f) - 1.0f, z = Rand(2.0f) - 1.0f;
		//	Vector3 dir(x, y, z);
		//	NORMALIZE(dir);
		//	float dis = 0.0f;
		//	PhotonTrace(Photon(lightCentre, dir, Color(1.0f, 1.0f, 1.0f)),
		//		1, 1.0f, dis);
		//	//std::cout << PhotonCount << std::endl;
		//}
	}

	void Engine::buildTree()
	{
		printf("build tree\n");
		printf("list size : %d\n", photonList.size());
		std::vector<Photon>::iterator start;
		start = photonList.begin();
		this->photonTree->setIter(start);
		this->photonTree->build(0, 0, photonList.size(), photonTree->root, photonList);
	}

	Primitive* Engine::Raytrace(Ray& a_Ray, Color& a_Acc, int a_Depth, float a_RIndex, float& a_Dist)//edit 起初记录N
	{
		if (a_Depth > TRACEDEPTH)return 0;
		a_Dist = 1000000.0f;
		Vector3 pi;
		Primitive* prim = 0;
		int result;

		//for (int i = 0; i < m_Scene->GetNrPrimitives(); ++i)
		//{
		//	Primitive* pr = m_Scene->GetPrimitive(i);
		//	int res;
		//	if (res = pr->Intersect(a_Ray, a_Dist))
		//	{
		//		prim = pr;
		//		result = res;
		//	}
		//}

		getCollide(a_Ray, a_Dist, result, prim);

		if (!prim)return 0;
		if (prim->IsLight())
		{
			a_Acc = Color(1, 1, 1);
		}
		else
		{
			pi = a_Ray.GetOrigin() + a_Ray.GetDirection()*a_Dist;

			Vector3 N = prim->GetNormal(pi);

			//for (int k = 0; k < m_Scene->GetNrPrimitives(); ++k)
			//{
			//	Primitive* temp = m_Scene->GetPrimitive(k);
			//	if (temp->IsLight())
			//	{
			//		//printf("Calculating diffuse...\n");
			//		Primitive* light = temp;
			//		//阴影
			//		float shadow = 1.0f;
			//		if (light->GetType() == Primitive::SPHERE)
			//		{
			//			Vector3 L = ((Sphere*)light)->GetCentre() - pi;
			//			float dis = LENGTH(L);
			//			L *= (1.0f / dis);
			//			Ray ray(pi + L*EPSILON, L);
			//			for (int u = 0; u < m_Scene->GetNrPrimitives(); ++u)
			//			{
			//				Primitive* temp = m_Scene->GetPrimitive(u);
			//				if ((temp != light) && (temp->Intersect(ray, dis)))
			//				{
			//					shadow = 0;
			//					break;
			//				}
			//			}
			//		}
			//		//if (shadow > 0)
			//		//{
			//		//	//漫反射 pm中改用光子图，不用判断是否遮挡
			//		//	Vector3 L = ((Sphere*)light)->GetCentre() - pi;
			//		//	NORMALIZE(L);
			//		//	//Vector3 N = prim->GetNormal(pi);
			//		//	if (prim->GetMaterial()->GetDiffuse() > 0)
			//		//	{
			//		//		float dotrlt = DOT(L, N);
			//		//		if (dotrlt > 0)
			//		//		{
			//		//			//printf("add\n");
			//		//			float rdiff = dotrlt*prim->GetMaterial()->GetDiffuse()*shadow;
			//		//			a_Acc += rdiff*prim->GetColor(pi)*light->GetColor(pi);
			//		//		}
			//		//	}
			//		//	
			//		//}
			//	}
			//}
			//return 0;

			//漫反射

			float r2 = RR;
			Color clIndex = photonTree->getColorIndex(pi, a_Ray.GetDirection(), r2,
												prim->GetNormal(pi));
			float para = ZOOMFACTOR/(r2*PI*PHOTON);
			a_Acc += prim->GetColor(pi)*para*clIndex;

			//printf("clIndex: %g,%g,%g\n", clIndex.x, clIndex.y, clIndex.z);
			//printf("after diffuse %g,%g,%g\n", a_Acc.x, a_Acc.y, a_Acc.z);



			//反射光
			float refl = prim->GetMaterial()->GetReflection();
			if ((refl > 0.0f)&&(a_Depth<TRACEDEPTH))
			{
				//printf("反射\n");
				//Vector3 normal = prim->GetNormal(pi);
				Vector3 R = a_Ray.GetDirection() - 2.0f * DOT(a_Ray.GetDirection(), N)*N;
				Color refl_color(0, 0, 0);
				float dist;
				Raytrace(Ray(pi + R*EPSILON, R), refl_color, a_Depth + 1, a_RIndex, dist);
				a_Acc += refl*refl_color*prim->GetColor(pi);
				
			}
			//return 0;

			//折射光
			float refr = prim->GetMaterial()->GetRefraction();
			if ((refr > 0) && (a_Depth < TRACEDEPTH))//注意能折射一定是球，而且纯色无贴图
			{
				//if (result > 0)
				//{
				//	printf("get in refr\n");
				//}
				//else
				//{
				//	printf("go out refr\n");
				//}

				float rindex = prim->GetMaterial()->GetRefractionIndex();
				//printf("%g/%g\n", a_RIndex, rindex);
				if (result < 0)rindex = 1.0f;
				float n = a_RIndex / rindex;
				//float n = rindex / a_RIndex;
				Vector3 NN = N * (float)result;
				float cosI = -DOT(NN, a_Ray.GetDirection());
				float cosT2 = 1.0f - n * n * (1.0f - cosI * cosI);
				if (cosT2 > 0.0f)
				{
					Vector3 T = (n * a_Ray.GetDirection()) + (n * cosI - sqrtf(cosT2)) * NN;
					Color rcol(0, 0, 0);
					float dist;
					Raytrace(Ray(pi + T * EPSILON, T), rcol, a_Depth + 1, rindex, dist);
					// apply Beer's law
					Color absorbance = prim->GetMaterial()->GetAbsorb() * 0.15f * -dist;
					Color transparency = Color(expf(absorbance.r), expf(absorbance.g), expf(absorbance.b));
					//printf("transparency : %g, %g, %g\n", transparency.x, transparency.y, transparency.z);
					//printf("rcol : %g, %g, %g", rcol.x, rcol.y, rcol.z);
					a_Acc += rcol*transparency*refr;
					//printf("a_Acc: %g, %g, %g\n", a_Acc.x, a_Acc.y, a_Acc.z);
				}
			}

		}
		return prim;
	}

	void Engine::Render()
	{
		//Vector3 origin(0, 0, -10);
		//float zpos = m_Screen->get_Z();
		int wid = m_Screen->pixel_width;
		int hgt = m_Screen->pixel_height;
		int row_offset = m_Screen->pixel_offset_w;
		int col_offset = m_Screen->pixel_offset_h;
		float delta_row = m_Screen->delta_width;
		float delta_col = m_Screen->delta_height;


		Vector3 xStep = delta_row*m_Screen->axis_width;
		Vector3 yStep = delta_col*m_Screen->axis_height;
		// - 2 * xStep + 4 * yStep;
		Vector3 Start = m_Screen->centre
			+ (-row_offset)*xStep
			+ (-col_offset)*yStep;
		Vector3 temp = Start;
		
		printf("row off %d, col off %d\n", row_offset, col_offset);
		int count = 0;
		for (int y = hgt-1; y >= 0; --y)
		{
			for (int x = 0; x < wid; ++x)
			{
				//中心是temp
				Color a_Acc(0, 0, 0);
				float ret = 1.0f / 9.0f;
				for (int i = -1; i < 2; ++i)
				{
					for (int j = -1; j < 2; ++j)
					{
						Vector3 point = temp + float(i)*0.5f*xStep + float(j)*0.5f*yStep;
						Vector3 dir = point - m_Screen->viewPoint;
						NORMALIZE(dir);
						Ray ray(m_Screen->viewPoint, dir);
						float a_Dist = 0.0f;
						Color cc(0, 0, 0);
						Primitive* prim = Raytrace(ray, cc, 1, 1.0f, a_Dist);
						a_Acc += cc*ret;
					}
				}

				//Vector3 dir = temp - m_Screen->viewPoint;
				//NORMALIZE(dir);
				//Ray ray(m_Screen->viewPoint, dir);
				//float a_Dist = 0.0f;
				//
				//Primitive* prim = Raytrace(ray, a_Acc, 1, 1.0f, a_Dist);
				
				m_Screen->Painting(y, x, a_Acc);
				temp += xStep;
				//exit(0);
			}
			Start += yStep;
			temp = Start;
			printf("linecount : %d\r", count++);
		}
		printf("\nComplete!\n");
	}
}