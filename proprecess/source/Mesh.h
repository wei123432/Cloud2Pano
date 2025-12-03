#pragma once
#include <vector>
#include <Eigen/Dense>


struct Indices
{
	int i0, i1, i2;
};

struct Mesh
{
	std::vector<Eigen::Vector3d> V;
	std::vector<Indices> F;
};