#ifndef TRI_KD_H
#define TRI_KD_H

#include "vector3.h"
#include "scene.h"

namespace RayTracer
{
	class Tri_kdNode
	{
	public:
		aabb box;
		//std::vector<int> triList;
		Tri_kdNode* lc;
		Tri_kdNode* rc;
		Tri_kdNode* father;
		int split;

		//int split;
		std::vector<int> triList;
		Tri_kdNode() :lc(0), rc(0), split(0) { triList.clear(); }

		int GetSize()
		{
			return triList.size();
		}

		void push(int i)
		{
			triList.push_back(i);
		}
		void clear()
		{
			triList.clear();
		}
		void SetBox(Vector3& a_Pos, Vector3& a_Size)
		{
			box.Set(a_Pos, a_Size);
		}
	};

	class Tri_Kdtree
	{
	public:
		Tri_kdNode* root;
		int node_count;
		int height;
		//std::vector<Triangle>::iterator start;
		std::vector<int> search_rlt;

	public:
		Tri_Kdtree();

		//void setIter(std::vector<Triangle>::iterator& iter)
		//{
		//	start = iter;
		//}

		void build(int layer, Tri_kdNode*& parent);

		void clear()
		{
			search_rlt.clear();
		}

		void search(Ray& a_Ray, Tri_kdNode* root, float& a_Dist);

		void GetTriList(Ray& a_Ray, float& a_Dist)
		{
			clear();
			search(a_Ray, root, a_Dist);
		}
		void Init()
		{
			build(0, root);
		}
	};
}

#endif // !TRI_KD_H
