#ifndef KDTREE_H
#define KDTREE_H

#include "hitpoint.h"
#include <cstdlib>
#include <algorithm>
#include <vector>

namespace RayTracer
{
	class TreeNode
	{
	public:
		
		Photon* photon;
		int split; // 0 x 1 y 2 z
		TreeNode* lc;
		TreeNode* rc;
		float minx, miny, minz;
		float maxx, maxy, maxz;
		//HitPoint* parent;
	public:
		TreeNode() { lc = NULL; rc = NULL; }
		TreeNode(Photon* hpt):photon(hpt)
		{
			lc = NULL; rc = NULL;// parent = NULL;
		}
		void setPtn(Photon* ptn) { photon = ptn; }
		//bool operator < (const TreeNode& other) const;
	};

	class KdTree
	{
		//std::vector<TreeNode*> NodeList;
	public:
		TreeNode* root;
		int node_count;
		int height;
		std::vector<Photon>::iterator start;
		std::vector<TreeNode*> search_rlt;
	public:
		KdTree() :node_count(0), height(0) {
			root = NULL;
			search_rlt.clear();
		}

		//void push(TreeNode* pointer);
		void setIter(std::vector<Photon>::iterator& iter)
		{
			start = iter;
		}

		void build(int layer, int begin, int end, TreeNode*& parent,std::vector<Photon>& vec);
		
		void search(Vector3& dst, float& distance2, TreeNode* root)
		{
			if (!root)return;
			float dis2 = 0.0f;

			if (dst.x < root->minx)dis2 += (root->minx - dst.x)*(root->minx - dst.x);
			else if (dst.x > root->maxx)dis2 += (dst.x - root->maxx)*(dst.x - root->maxx);
			if (dst.y < root->miny)dis2 += (root->miny - dst.y)*(root->miny - dst.y);
			else if (dst.y > root->maxy)dis2 += (dst.y - root->maxy)*(dst.y - root->maxy);
			if (dst.z < root->minz)dis2 += (root->minz - dst.z)*(root->minz - dst.z);
			else if (dst.z > root->maxz)dis2 += (dst.z - root->maxz)*(dst.z - root->maxz);

			if (dis2 > distance2)
			{
				return;
			}
			//within R

			//check current point
			float r2 = (root->photon->ori.x - dst.x)*(root->photon->ori.x - dst.x)
				+ (root->photon->ori.y - dst.y)*(root->photon->ori.y - dst.y)
				+ (root->photon->ori.z - dst.z)*(root->photon->ori.z - dst.z);

			if (r2 < distance2)
			{
				search_rlt.push_back(root);
			}

			search(dst, distance2, root->lc);
			search(dst, distance2, root->rc);
			
		}

		Color getColorIndex(Vector3& Pos, Vector3& viewDir, float dis2, Vector3& N)
		{
			Color colorIndex(0, 0, 0);
			search(Pos, dis2, this->root);
			//printf("photon count : %d\n", search_rlt.size());
			for (int i = 0; i < search_rlt.size(); ++i)
			{
				//对每个光子结合方向算强度
				Vector3 L = (-1)*search_rlt[i]->photon->dir;
				float viewDot = DOT(L, N);
				//Vector3 R = L - 2.0f*DOT(L, N)*N;
				//float viewDot = DOT(viewDir, R);
				if(viewDot>0.0f)colorIndex += viewDot*search_rlt[i]->photon->color;
			}
			//colorIndex *= 1.0f / search_rlt.size();
			this->clear();
			return colorIndex;
		}


		void clear()
		{
			search_rlt.clear();
		}

		void destroy(TreeNode* root)
		{
			if (root->lc)
			{
				destroy(root->lc);
			}
			if (root->rc)
			{
				destroy(root->rc);
			}

			delete root;
			root = NULL;
			
		}
	};

}
#endif // !KDTREE_H
