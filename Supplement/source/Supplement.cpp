#define _USE_MATH_DEFINES
#include <iostream>
#include <sstream>
#include <filesystem>
#include <algorithm>
#include"Camera.h"
#include"Algorithm.h"
#include"Mesh.h"
#include"Loader.h"
#include <cmath>
#include <unordered_map>

// 从原始模型中提取位于一定限制的模型
Mesh extractSubMeshByAABB(
    const Mesh& fullMesh,
    const AABB& box,
    const std::vector<int>& candidateTris)
{
    Mesh result;
    result.V.clear();
    result.F.clear();

    std::unordered_map<int, int> oldToNew;
    oldToNew.reserve(candidateTris.size() * 3);

    auto getOrAddVertex = [&](int oldIdx) -> int
        {
            auto it = oldToNew.find(oldIdx);
            if (it != oldToNew.end())
                return it->second;

            int newIdx = static_cast<int>(result.V.size());
            result.V.push_back(fullMesh.V[oldIdx]);
            oldToNew[oldIdx] = newIdx;
            return newIdx;
        };

    for (int triIdx : candidateTris)
    {
        const auto& f = fullMesh.F[triIdx];

        const Eigen::Vector3d& v0 = fullMesh.V[f.i0];
        const Eigen::Vector3d& v1 = fullMesh.V[f.i1];
        const Eigen::Vector3d& v2 = fullMesh.V[f.i2];

        // 过滤策略：至少一个顶点在包围盒内就保留
        bool keep =
            contains(box, v0) ||
            contains(box, v1) ||
            contains(box, v2);

        if (!keep) continue;

        // 顶点索引压缩
        Indices newFace;
        newFace.i0 = getOrAddVertex(f.i0);
        newFace.i1 = getOrAddVertex(f.i1);
        newFace.i2 = getOrAddVertex(f.i2);
        result.F.push_back(newFace);
    }
    return result;
}

int main()
{
    //加载原始模型包围盒
    const std::filesystem::path obj_path = "D:\\experience\\Web\\data_origin\\3DModel\\Simplication_OBJ\\Data";
    std::vector<std::filesystem::path> obj_files = findAllOBJFiles(obj_path);
    std::vector<Mesh> meshes(obj_files.size());
    std::vector<char> loaded(obj_files.size(), 0);
    std::vector<BVH> bvhs(meshes.size());

#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < (int)obj_files.size(); ++i)
    {
        Mesh current_mesh;
        if (loadOBJ(obj_files[i].string(), current_mesh))
        {
            meshes[i] = std::move(current_mesh);
            loaded[i] = 1;
            //写入模型后计算包围盒信息
            if (!loaded[i]) continue;
            bvhs[i].build(&meshes[i]);
#pragma omp critical
            std::cout << "BVH nodes: " << bvhs[i].nodes.size() << std::endl;
        }
        else {
#pragma omp critical
            std::cerr << "Failed to load OBJ file: " << obj_files[i] << std::endl;
        }
    }

	//对每个可视模型加载包围盒和2D可视掩码
    const std::filesystem::path visual_origin_path = "D:\\experience\\Web\\data_middle\\VisualModel\\new_visual";
    const std::filesystem::path visual_final_path = "D:\\experience\\Web\\data_middle\\VisualModel\\new_visual_supply";
    std::filesystem::create_directories(visual_final_path);
	std::vector<std::filesystem::path> visual_files = findAllOBJFiles(visual_origin_path);

#pragma omp parallel for schedule(dynamic)
    for (int k = 0; k < (int)visual_files.size(); ++k)
    {
        Mesh visibleMesh;
        if (!loadOBJNoSRS(visual_files[k].string(), visibleMesh))
        {
#pragma omp critical
            std::cerr << "Failed to load visible mesh obj." << std::endl;
            continue;
        }
       
		//计算可视模型包围盒和2D掩码
        AABB visibleBox = computeMeshBox(visibleMesh);
        OccupancyGrid2D visibleMask;
        double visZMin, visZMax;
        buildVisibleMaskXY(visibleMesh, visibleMask, visZMin, visZMax);

		//接收裁切后的完整模型
        Mesh fullMesh;
        fullMesh.V.clear();
        fullMesh.F.clear();

        for (int j = 0; j < static_cast<int>(obj_files.size()); ++j)
        {
            if (!loaded[j]) continue;
            const BVH& bvh = bvhs[j];
            if (bvh.nodes.empty())
                continue;

            //如果瓦块根节点与可视包围盒没有交集，则跳过
            const AABB& tileBox = bvh.nodes[0].box;
            /*std::cout << "BVH nodes: " << bvhs[j].nodes.size() << ", Tile Box Min: "
                << tileBox.bmin.transpose() << ", Max: " << tileBox.bmax.transpose() << std::endl;*/
           
            if (!overlaps(tileBox, visibleBox))
                continue;

            //构建占用网络
            std::vector<int> candidateTris;
            candidateTris.reserve(static_cast<int>(meshes[j].F.size()));
            queryBVH_AABB(bvh, 0, visibleBox, candidateTris);
            if (candidateTris.empty())
                continue;

            std::vector<int> filteredTris;
            filteredTris.reserve(candidateTris.size());

            for (int triIdx : candidateTris)
            {
                const auto& f = meshes[j].F[triIdx];

                const Eigen::Vector3d& v0 = meshes[j].V[f.i0];
                const Eigen::Vector3d& v1 = meshes[j].V[f.i1];
                const Eigen::Vector3d& v2 = meshes[j].V[f.i2];

                auto testVertex = [&](const Eigen::Vector3d& v) -> bool
                    {
                        // 高度约束（严格贴 z 范围）
                        if (v.z() < visZMin || v.z() > visZMax)
                            return false;

                        // XY 是否落在“可视区域 mask”里？
                        return visibleMask.isOccupied(v.x(), v.y());
                    };

                bool keep = testVertex(v0) || testVertex(v1) || testVertex(v2);
                if (keep)
                    filteredTris.push_back(triIdx);
            }
            if (filteredTris.empty())
                continue;

            Mesh croppedMesh = extractSubMeshByAABB(meshes[j], visibleBox, filteredTris);
            if (croppedMesh.F.empty())
                continue; // 没有实际落入盒中的三角形，跳过
            int vOffset = static_cast<int>(fullMesh.V.size());
            fullMesh.V.insert(fullMesh.V.end(), croppedMesh.V.begin(), croppedMesh.V.end());

            for (const auto& f : croppedMesh.F)
            {
                Indices nf;
                nf.i0 = f.i0 + vOffset;
                nf.i1 = f.i1 + vOffset;
                nf.i2 = f.i2 + vOffset;
                fullMesh.F.push_back(nf);
            }
            std::cout << "Tile " << j << " contributed "
                << croppedMesh.F.size() << " faces to merged mesh." << std::endl;
        }

        const std::filesystem::path save_path = visual_final_path / visual_files[k].filename();
        if (!saveOBJ(save_path.string(), fullMesh))
        {
#pragma omp critical
            std::cerr << "Failed to save merged cropped mesh." << std::endl;
			continue;
        }
#pragma omp critical
        std::cout << "[VIS] Saved: "<< save_path
            << "Vertices: " << fullMesh.V.size()
            << ", Faces: " << fullMesh.F.size() << std::endl;
    }

	return 0;
}