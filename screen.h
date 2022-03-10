#ifndef SCREEN_H
#define SCREEN_H

#include "vector3.h"
#include <opencv2\opencv.hpp>

namespace RayTracer {
	class Screen
	{
	public:
		Vector3 viewPoint;
		Vector3 axis_depth;
		Vector3 axis_height;
		Vector3 axis_width;
		Vector3 centre;
		float f;

		int pixel_width;
		int pixel_height;
		cv::Mat* mat;
		//float screen_width;
		//float screen_height;
		float delta_width;
		float delta_height;
		int pixel_offset_w;
		int pixel_offset_h;
		float z_Pos;
	public:
		Screen(Vector3& ori,Vector3& depthDir,Vector3& heightDir, int width, int height, float jiaoju):pixel_width(width),pixel_height(height)
		{
			f = jiaoju;
			viewPoint = ori;
			//z_Pos = -5.0f;
			//screen_width = width / 100.0f;
			//screen_height = height / 100.0f;
			NORMALIZE(depthDir);
			NORMALIZE(heightDir);
			axis_width = heightDir.Cross(depthDir);
			NORMALIZE(axis_width);
			axis_depth = depthDir;
			axis_height = heightDir;

			centre = viewPoint + f*axis_depth;

			delta_width = 1 / 100.0f;
			delta_height = 1 / 100.0f;
			pixel_offset_w = width / 2;
			pixel_offset_h = height / 2;
			mat = new cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
		}


		void Painting(int row, int col, Color color)
		{
			int b, g, r = 0;
			r = color.x * 256;
			g = color.y * 256;
			b = color.z * 256;
			//if (row == pixel_height - 21 && col == 20)
			//{
			//	printf("canvas %d, %d, %d\n", r, g, b);
			//}
			if (b > 255)b = 255;
			if (g > 255)g = 255;
			if (r > 255)r = 255;
			mat->at<cv::Vec3b>(row, col) = cv::Vec3b(b, g, r);
		}
	};
}

#endif // !SCREEN_H

