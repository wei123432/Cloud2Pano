#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <string>
#include <filesystem>

struct GLMesh 
{
	unsigned int vao = 0;
	unsigned int vbo = 0;
	unsigned int ibo = 0;
	int indexCount = 0; //三角形索引个数
	uint32_t baseTriIdOffset = 0; // 全局三角形偏移
	std::vector<glm::vec3> positions; //原始顶点
	std::vector<uint32_t>  indices; // 索引坐标
};

struct LoadedScene {
	std::vector<GLMesh> meshes;
	// 用于全局ID->(mesh, localTriIndex) 回溯
	std::vector<std::pair<int, int>> idToMeshTri; 
};

LoadedScene loadObjFolderBuildGL(const std::string& folder);