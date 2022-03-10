#include "kdtree.h"
#include <cstdlib>

namespace RayTracer
{
	//int TreeNode::layer = 0;
	std::vector<Photon> photonList;
	int Photon::layer = 0;

	void KdTree::build(int layer, int first, int last, TreeNode*& parent, std::vector<Photon>& vec)
	{
		if (first >= last)return;
		Photon::layer = layer % 3;
		//printf("building layer %d\n", layer);
		int mid = (first + last) / 2;
		std::nth_element(start + first, start + mid, start + last);

		parent = new TreeNode(&(vec[mid]));
		
		//parent->setPtn(&(vec[mid]));

		parent->split = layer;
		build(layer + 1, first, mid, parent->lc, vec);
		build(layer + 1, mid + 1, last, parent->rc, vec);

		parent->maxx = parent->photon->ori.x;
		parent->minx = parent->photon->ori.x;
		parent->maxy = parent->photon->ori.y;
		parent->miny = parent->photon->ori.y;
		parent->maxz = parent->photon->ori.z;
		parent->minz = parent->photon->ori.z;
		if (parent->lc)
		{
			parent->maxx = std::max(parent->maxx, parent->lc->maxx);
			parent->maxy = std::max(parent->maxy, parent->lc->maxy);
			parent->maxz = std::max(parent->maxz, parent->lc->maxz);

			parent->minx = std::min(parent->minx, parent->lc->minx);
			parent->miny = std::min(parent->miny, parent->lc->miny);
			parent->minz = std::min(parent->minz, parent->lc->minz);
		}
		if (parent->rc)
		{
			parent->maxx = std::max(parent->maxx, parent->rc->maxx);
			parent->maxy = std::max(parent->maxy, parent->rc->maxy);
			parent->maxz = std::max(parent->maxz, parent->rc->maxz);
														  
			parent->minx = std::min(parent->minx, parent->rc->minx);
			parent->miny = std::min(parent->miny, parent->rc->miny);
			parent->minz = std::min(parent->minz, parent->rc->minz);
		}
	}
}