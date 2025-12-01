 #pragma once

#include "Camera.h"
#include <Eigen/Dense>
class Cloud
{
public:
	Cloud() {};
	//简化模型的输入点云
	Eigen::Vector3d SimpleModelPoint;
	//相机的世界坐标点
	Eigen::Vector4d CameraPosition;
	//全景图的长宽
	void Panoprojection(Eigen::Vector3d v,const Camera& camera);
};


