#include "tri_kd.h"

namespace RayTracer
{
	int Triangle::layer = 0;
	extern std::vector<Triangle> triangles;
	extern std::vector<Vector3> vertices;

	bool cmp(int& a, int& b)
	{
		//return a < b;
		return triangles[a] < triangles[b];
	}

	Tri_Kdtree::Tri_Kdtree() : node_count(0), height(0)
	{
		search_rlt.clear();
		root = new Tri_kdNode();
		root->father = NULL;
		int tricount = triangles.size();
		for (int i = 0; i < tricount; ++i)
		{
			root->push(i);
		}
	}

	void Tri_Kdtree::build(int layer, Tri_kdNode*& parent)
	{
		//��root��ʼ���죬root�Ѿ�����������Ƭ�ı�ţ�����ȡ��ֵ���������ֵƽ�潫��Ƭ�ֳ�����

		Triangle::layer = layer % 3;
		//printf("building layer %d\n", layer);
		int Size = parent->GetSize();//ԭ�򣬡�=5 ��������չ�½ڵ�
		//printf("Size == %d\n", Size);
		//if (Size == 0)
		//{
		//	int mmm = 0;
		//}
		//Tri_kdNode* father = parent->father;
		if (Size <= 64)//father && (Size == father->GetSize()||Size == 0)
		{
			//������չ�ӽڵ�,������ӳߴ��ֱ�ӷ��ؼ���
			//���ں��ӳߴ�ļ��㣬������ǰ������������Ƭ����
			if (parent->triList.empty())return;
			float Max[3], Min[3];
			Max[0] = -100000, Max[1] = -100000, Max[2] = -100000;
			Min[0] = 100000, Min[1] = 100000, Min[2] = 100000;
			
			for (int la = 0; la < 3; ++la)
			{
				for (int i = 0; i < Size; ++i)
				{
					float a[3], ma, mi;
					a[0] = vertices[triangles[parent->triList[i]].v0].cell[la];
					a[1] = vertices[triangles[parent->triList[i]].v1].cell[la];
					a[2] = vertices[triangles[parent->triList[i]].v2].cell[la];
					//�������ĳһά����
					ma = std::max({ a[0],a[1],a[2] });
					mi = std::min({ a[0],a[1],a[2] });
					//�õ���������ĳһά�Ŀ��
					Max[la] = std::max(Max[la], ma);
					Min[la] = std::min(Min[la], mi);
				}
			}
			
			parent->box.Set(Vector3(Min[0], Min[1], Min[2]), Vector3(Max[0], Max[1], Max[2]), 1);
			return;
		}
		else
		{
			//��չ�ӽڵ�
			int mid = Size / 2;
			std::vector<int>::iterator start = parent->triList.begin();
			std::nth_element(start, start + mid, start + Size, cmp);//����layerΪ0����ôȡvectormid ��xƽ��

			parent->lc = new Tri_kdNode();
			parent->rc = new Tri_kdNode();
			parent->lc->father = parent;
			parent->rc->father = parent;

			int ly = Triangle::layer;
			float seb = triangles[parent->triList[mid]].m_Centre.cell[ly];
			//float seb = vertices[triangles[parent->triList[mid]].v0].cell[ly];
			for (int i = 0; i < Size; ++i)//i �������εı��
			{
				int flag1 = 0, flag2 = 0;
				if (i == 10) 
				{ 
					int pp = 0; 
				}
				float a[3];
				a[0] = vertices[triangles[parent->triList[i]].v0].cell[ly];
				a[1] = vertices[triangles[parent->triList[i]].v1].cell[ly];
				a[2] = vertices[triangles[parent->triList[i]].v2].cell[ly];

				for (int j = 0; j < 3; ++j)
				{
					if (a[j] < seb)flag1 = 1;
					else flag2 = 1;
				}

				if (flag1)parent->lc->push(parent->triList[i]);
				if (flag2)parent->rc->push(parent->triList[i]);
			}


			build(layer + 1, parent->lc);
			build(layer + 1, parent->rc);
		}
		
		//parent->split = layer;//�ƺ�������Ҫ��¼ÿ���������ĸ���ָ��

		//��������ӳߴ磬�ж����������޺��Ӳ��Һ��ӷǿ�
		float maxx = -100000, maxy = -100000, maxz = -100000, 
			minx = 100000, miny = 100000, minz = 100000;
		if (parent->lc && (!parent->lc->triList.empty()))
		{
			Vector3 v1 = parent->lc->box.GetPos();
			Vector3 v2 = v1 + parent->lc->box.GetSize();
			maxx = v2.x; maxy = v2.y; maxz = v2.z;
			minx = v1.x; miny = v1.y; minz = v1.z;
		}
		if (parent->rc&&(!parent->rc->triList.empty()))
		{
			Vector3 v1 = parent->rc->box.GetPos();
			Vector3 v2 = v1 + parent->rc->box.GetSize();
			maxx = std::max(maxx, v2.x);
			maxy = std::max(maxy, v2.y);
			maxz = std::max(maxz, v2.z);
		
			minx = std::min(minx, v1.x);
			miny = std::min(miny, v1.y);
			minz = std::min(minz, v1.z);
		}
		parent->box.Set(Vector3(minx, miny, minz), Vector3(maxx, maxy, maxz), 1);
	}


	void Tri_Kdtree::search(Ray& a_Ray, Tri_kdNode* root, float& a_Dist)//�����ܵ�С���ӵ���Ƭ��������
	{
		if (!root) {
			printf("Warning: Null Node\n");
			return;
		}
		if (root->triList.empty()) { return; }

		if (root->box.InterSect(a_Ray, a_Dist))//�Ͱ�Χ���н�
		{
			if (root->lc)//���ӽڵ�
			{
				search(a_Ray, root->lc, a_Dist);
				search(a_Ray, root->rc, a_Dist);
			}
			else  //�Ѿ���Ҷ�ӽڵ���,�����������нڵ����
			{
				for (int i = 0; i < root->GetSize(); ++i)
				{
					search_rlt.push_back(root->triList[i]);
				}
			}
		}
	}
}