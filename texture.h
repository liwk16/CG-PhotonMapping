#ifndef TEXTURE_H
#define TEXTURE_H

#include "vector3.h"
#include <opencv2\opencv.hpp>
#include <string>
namespace RayTracer
{
	class Texture
	{
		cv::Mat texture;
		std::string pathPrefix;
		int height;
		int width;

	public:
		Texture();
		void setImage(std::string pic);
		Vector3 getColor(int u, int v);
	};
}
#endif // !TEXTURE_H

