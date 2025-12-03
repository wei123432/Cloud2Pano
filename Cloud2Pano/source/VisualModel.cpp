#define _USE_MATH_DEFINES
#define X_SRS -60.590000
#define Y_SRS -19.490000
#define Z_SRS -6.190000
#include <fstream>
#include <limits>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <algorithm>
#include <omp.h>
#include "Camera.h"
#include "Algorithm.h"
#include "Mesh.h"
#include "Loader.h"

//辅助循环体
struct RayHit 
{
    double t = std::numeric_limits<double>::infinity();
    int mesh_idx = -1;
    int tri_idx = -1;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    bool hit = false;
};

//合并多个顶点写成单一obj文件
static bool writeVisibleFacesOBJMulti(const std::string& path,const std::vector<Mesh>& meshes,
	const std::vector<std::vector<char>>& visibles) //visibles用于标记每个mesh的可视面（一层：模型，二层：面）
{
    std::ofstream ofs(path);
    if (!ofs) 
        return false;
    ofs.setf(std::ios::fixed);
    ofs.precision(6);
    ofs << "o VisibleFacesAll\n";

    size_t global_v_offset = 0;
    for (size_t mi = 0; mi < meshes.size(); ++mi) 
    {
        const Mesh& m = meshes[mi];
        if (mi >= visibles.size()) continue;
        const auto& vis = visibles[mi];
        if (vis.empty()) continue;

        // 标记用到的顶点
        std::vector<int> usedV(m.V.size(), 0);
        size_t usedFaceCount = 0;
        for (size_t fi = 0; fi < vis.size(); ++fi) 
        {
            if (!vis[fi]) continue;
            usedFaceCount++;
            usedV[m.F[fi].i0] = 1;
            usedV[m.F[fi].i1] = 1;
            usedV[m.F[fi].i2] = 1;
        }
        if (usedFaceCount == 0) continue;

 //        分组名（可选）
        /*std::string gname = (mi < obj_files.size() ? obj_files[mi].filename().string() : ("mesh_" + std::to_string(mi)));
        ofs << "g " << gname << "\n";*/

        // old -> new 索引映射
        std::vector<int> old2new(m.V.size(), -1);
        size_t local_count = 0;
        for (size_t vi = 0; vi < m.V.size(); ++vi) 
        {
            if (usedV[vi]) 
            {
                old2new[vi] = static_cast<int>(global_v_offset + local_count);
                const auto& v = m.V[vi];
                ofs << "v " << v.x() << " " << v.y() << " " << v.z() << "\n";
                local_count++;
            }
        }

        // 写面
        for (size_t fi = 0; fi < vis.size(); ++fi) 
        {
            if (!vis[fi]) continue;
            int a = old2new[m.F[fi].i0];
            int b = old2new[m.F[fi].i1];
            int c = old2new[m.F[fi].i2];
            ofs << "f " << (a + 1) << " " << (b + 1) << " " << (c + 1) << "\n";
        }
        global_v_offset += local_count;
    }
    return true;
}

int main()
{
	//加载模型并计算包围盒信息
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

    // Camera位姿
    std::vector<std::vector<double>> translations; 
    std::vector<std::vector<double>> quaternions;
    const std::string cameraPos_file = "D:\\experience\\Web\\data_origin\\POS\\camera1_pose.asc"; 
    if(!readData(cameraPos_file, translations, quaternions)) 
        return 1;
    for (size_t ci = 0; ci < translations.size(); ++ci)
    {
        Eigen::Vector3d cam_pos(translations[ci][0], translations[ci][1], translations[ci][2]);
        const auto& qv = quaternions[ci];
        Eigen::Quaterniond cam_quat(qv[3], qv[0], qv[1], qv[2]);
        Camera camera(cam_pos, cam_quat);   

        // 生成模型的采样方向
        std::vector<Eigen::Vector3d> directions = camera.GenerateSphereGridDirections();
        std::cout << "Directions: " << directions.size() << "\n";
        std::vector<Eigen::Vector3d> ndirs;
        ndirs.resize(directions.size());

#pragma omp parallel for schedule(static)
        for (int i = 0; i < (int)directions.size(); ++i)
        {
            ndirs[i] = directions[i].normalized();
        }

        auto start = std::chrono::high_resolution_clock::now();
        //多瓦快组成的单模型寻找可视面与交点
        const bool BACKFACE_CULL = true;
        const int N = static_cast<int>(ndirs.size());
        std::vector<RayHit> hits(N);
#pragma omp parallel for schedule(dynamic)        
        for (int k = 0; k < ndirs.size(); ++k)
        {
            int best_mesh = -1;
            int best_tri = -1;
            Eigen::Vector3d best_p = Eigen::Vector3d::Zero();
            double best_t = std::numeric_limits<double>::infinity();

            for (int i = 0; i < (int)meshes.size(); ++i)
            {
                if (!loaded[i]) continue;
                double t;
                int ti;
                if (traverseBVHFirstHit(meshes[i], bvhs[i], 0, cam_pos, ndirs[k], BACKFACE_CULL, t, ti))
                {

                    if (t < best_t)
                    {
                        best_t = t;
                        best_mesh = i;
                        best_tri = ti;
                        best_p = cam_pos + t * ndirs[k];
                    }
                }
            }
            //记录最后的t值、击中的模型信息、面索引、击中点、击中状态
            if (best_mesh > 0 && best_tri >= 0)
            {
                hits[k].t = best_t;
                hits[k].mesh_idx = best_mesh;
                hits[k].tri_idx = best_tri;
                hits[k].point = best_p;
                hits[k].hit = true;
            }
            else
            {
                hits[k].hit = false;
            }
        }

        std::vector<std::vector<char>> visibles(meshes.size());
        for (size_t i = 0; i < meshes.size(); ++i)
        {
            if (!loaded[i]) continue;
            visibles[i].assign(meshes[i].F.size(), 0);
        }
        std::vector<Eigen::Vector3d> allHitPoints;
        allHitPoints.reserve(N);
        for (int k = 0; k < N; ++k) {
            if (!hits[k].hit) continue;
            visibles[hits[k].mesh_idx][hits[k].tri_idx] = 1;
            allHitPoints.push_back(hits[k].point);
        }

        const auto out_root = std::filesystem::path("D:\\experience\\Web\\data_middle\\VisualModel\\new_visual");
        std::filesystem::create_directories(out_root);

		const std::string out_faces = (out_root / ("visible_cam_" + std::to_string(ci) + ".obj")).string();
        if (writeVisibleFacesOBJMulti(out_faces, meshes, visibles))
        {
#pragma omp critical
            std::cout << "[OK] Faces -> " << out_faces << "\n";
        }
        else
        {
#pragma omp critical
            std::cerr << "Failed to write faces: " << out_faces << "\n";
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "Function took " << duration.count() << " milliseconds to execute." << std::endl;
    }
}
