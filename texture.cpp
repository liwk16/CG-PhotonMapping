#include "texture.h"

namespace RayTracer
{
	Texture::Texture()
	{
		pathPrefix = "D:\\Grade22\\cv\\texture\\";
		texture = NULL;
		height = 0;
		width = 0;
	}
	void Texture::setImage(std::string pic)
	{
		pathPrefix += pic;
		texture = cv::imread(pathPrefix);
		height = texture.rows;
		width = texture.cols;
	}

	Vector3 Texture::getColor(int u, int v)
	{
		cv::Vec3b color;
		u %= height;
		v %= width;
		if (u < 0)u += height;
		if (v < 0)v += width;
		color = texture.at<cv::Vec3b>(u, v);
		//printf("%d,%d\n", u, v);
		return Vector3(color[2]/256.0f, color[1]/256.0f, color[0]/256.0f);
	}
}