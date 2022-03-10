#include "raytracer.h"
using namespace RayTracer;
#include <opencv2\opencv.hpp>
#include "screen.h"
#include "scene.h"
#include "kdtree.h"

int main()
{
	/*
	cv::Mat base(400, 300, CV_8UC3, cv::Scalar(255, 255, 255));
	cv::imshow("test", base);
	cv::waitKey(0);
	exit(0);*/

	//Vector3 x = Vector3(0, 0, -1).Cross(Vector3(0, 1, -1));
	//std::cout << x.x << ' ' << x.y << ' ' << x.z << std::endl;
	//
	//exit(0);
	Engine* engine = new Engine();
	Screen screen(Vector3(-2, 4, -10), Vector3(0, -0.4f, 1), Vector3(0, 1, 0.4f), 800, 600, 5.0f);
	//Screen screen(Vector3(0, 6, -8), Vector3(0, -0.6f, 0.8f), Vector3(0, 0.8f, 0.6f), 800, 600, 5.0f);//600*400 原来是
	engine->GetScene()->ReadObj("D:\\Grade22\\cv\\Model\\dragon.obj");

	//exit(0);
	engine->GetScene()->InitScene();
	//exit(0);
	engine->SetTarget(&screen);

	//
	std::cout << "Photon Map Building...\n";
	engine->PhotonShooting(Vector3(-5, 7, 0), PHOTON);
	engine->buildTree();
	std::cout << "Complete\n";

	std::cout << "Rendering...\n";
	engine->Render();
	
	//cv::imshow("img", *(screen.mat));
	//cv::waitKey(0);
	cv::imwrite("D:\\Grade22\\图形学\\testpic\\final_kang1.png", *(screen.mat));
}