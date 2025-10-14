#define _USE_MATH_DEFINES
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <cmath>
#include <filesystem>
#include <algorithm>
#include <numeric>
#include <omp.h>

#include "Camera.h"
#include "Pano.h"
#include "Algorithm.h"
#include "Mesh.h"

// 把obj的索引改为从0开始
static int parseIndex(const std::string& indice_string)
{
    size_t slash = indice_string.find('/');
    std::string s = (slash == std::string::npos) ? indice_string : indice_string.substr(0, slash);
    int idx = std::stoi(s);
    return idx - 1;
}

// 加载obj文件
static bool loadOBJ(const std::string& path, Mesh& mesh) {
    std::ifstream ifs(path);
    if (!ifs) return false;
    mesh.V.clear();
    mesh.F.clear();

    std::string line;
    std::vector<int> face;
    while (std::getline(ifs, line))
    {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;
        if (tag == "v")
        {
            double x, y, z;
            ss >> x >> y >> z;
            mesh.V.emplace_back(x, y, z);
        }
        else if (tag == "f") {
            face.clear();
            std::string token;
            while (ss >> token) {
                try {
                    face.push_back(parseIndex(token));
                }
                catch (...) {}
            }
            if (face.size() >= 3) {
                for (size_t j = 1; j + 1 < face.size(); ++j) {
                    mesh.F.push_back({ face[0], face[j], face[j + 1] });
                }
            }
        }
    }
    return true;
}

// 找到文件夹中所有的obj
static std::vector<std::filesystem::path> findAllOBJFiles(const std::filesystem::path& rootDir)
{
    std::vector<std::filesystem::path> objFiles;
    try {
        for (const auto& entry : std::filesystem::recursive_directory_iterator(rootDir)) {
            if (entry.is_regular_file()) {
                std::string ext = entry.path().extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
                if (ext == ".obj") {
                    objFiles.push_back(entry.path());
                }
            }
        }
    }
    catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "filesystem error: " << e.what() << std::endl;
    }

    return objFiles;
}

// 写入点数据
static bool writePointsOBJ(const std::string& path, const std::vector<Eigen::Vector3d>& pts) {
    std::ofstream ofs(path);
    if (!ofs) return false;
    ofs.setf(std::ios::fixed);
    ofs.precision(6);
    ofs << "o VisiblePoints\n";
    for (const auto& p : pts) {
        ofs << "v " << p.x() << " " << p.y() << " " << p.z() << "\n";
    }
    return true;
}

//写入可视面数据
static bool writeVisibleFacesOBJ(const std::string& path, const Mesh& m, const std::vector<char>& visible) {
    std::vector<int> usedFaces;
    for (size_t i = 0; i < visible.size(); ++i)
        if (visible[i])
            usedFaces.push_back((int)i);
    if (usedFaces.empty())
    {
        std::ofstream ofs(path); // write empty object header
        if (!ofs) return false;
        ofs << "o VisibleFaces\n";
        return true;
    }
    std::vector<int> usedVtxFlags(m.V.size(), 0);
    for (int ti : usedFaces)
    {
        usedVtxFlags[m.F[ti].i0] = 1;
        usedVtxFlags[m.F[ti].i1] = 1;
        usedVtxFlags[m.F[ti].i2] = 1;
    }
    std::vector<int> old2new(m.V.size(), -1);
    std::vector<Eigen::Vector3d> newV;
    newV.reserve(m.V.size());
    for (size_t vi = 0; vi < m.V.size(); ++vi)
    {
        if (usedVtxFlags[vi])
        {
            old2new[vi] = (int)newV.size();
            newV.push_back(m.V[vi]);
        }
    }
    std::ofstream ofs(path);
    if (!ofs) return false;
    ofs.setf(std::ios::fixed);
    ofs.precision(6);
    ofs << "o VisibleFaces\n";
    for (const auto& v : newV)
    {
        ofs << "v " << v.x() << " " << v.y() << " " << v.z() << "\n";
    }
    for (int ti : usedFaces)
    {
        int a = old2new[m.F[ti].i0];
        int b = old2new[m.F[ti].i1];
        int c = old2new[m.F[ti].i2];
        ofs << "f " << (a + 1) << " " << (b + 1) << " " << (c + 1) << "\n";
    }
    return true;
}

// 写入可视面的索引
static bool writeFaceIDs(const std::string& path, std::vector<char>& visible)
{
    std::ofstream ofs(path);
    if (!ofs) return false;
    for (size_t i = 0; i < visible.size(); ++i)
        if (visible[i]) ofs << i << "\n";
    return true;
}

int main()
{
    // Camera位姿
    const Eigen::Vector3d cam_pos(-0.22033, -0.106948, 1.532681);
    const Eigen::Quaterniond cam_quat(-0.122489, -0.679658, -0.135731, 0.710379);
    Camera camera(cam_pos, cam_quat);

    // 特殊区域加密参数
    Camera::GridRegion region1, region2, region3;
   /* region1.dir = Eigen::Vector3d(1.0, 0.0, 0.0);
    region1.sigma_deg = 45.0;
    region1.strength = 2.0;
    region2.dir = Eigen::Vector3d(1.0, 0.0, 0.0);
    region2.sigma_deg = 15.0;
    region2.strength = 3.0;*/
    std::vector<Camera::GridRegion> special_region = { region1,region2,region3 };
    std::vector<Eigen::Vector3d> directions = camera.GenerateSphereGridDirections();
    std::cout << "Directions: " << directions.size() << "\n";
    std::vector<Eigen::Vector3d> ndirs;
    ndirs.resize(directions.size());
#pragma omp parallel for schedule(static)
    for (int i = 0; i < (int)directions.size(); ++i)
    {
        ndirs[i] = directions[i].normalized();
    }

    const std::filesystem::path obj_path = "D:\\experience\\try\\DasModel\\3DModel\\OBJ\\Data\\Tile_+001_+000";
    std::vector<std::filesystem::path> obj_files = findAllOBJFiles(obj_path);
    std::vector<Mesh> meshes(obj_files.size());
    std::vector<char> loaded(obj_files.size(), 0);

#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < (int)obj_files.size(); ++i)
    {
        Mesh current_mesh;
        if (loadOBJ(obj_files[i].string(), current_mesh))
        {
            meshes[i] = std::move(current_mesh);
            loaded[i] = 1;
#pragma omp critical
            std::cout << "Loaded OBJ file: " << obj_files[i] << " with "
                << meshes[i].F.size() << " faces." << std::endl;
        }
        else {
#pragma omp critical
            std::cerr << "Failed to load OBJ file: " << obj_files[i] << std::endl;
        }
    }

    std::vector<BVH> bvhs(meshes.size());
#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < (int)meshes.size(); ++i)
    {
        if (!loaded[i]) continue;
        bvhs[i].build(&meshes[i]);
#pragma omp critical
        std::cout << "BVH nodes: " << bvhs[i].nodes.size() << std::endl;
    }


    const bool BACKFACE_CULL = true;
    const auto out_dir = std::filesystem::path("D:\\experience\\try\\Visualmodel\\test1");
    std::filesystem::create_directories(out_dir);

#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < (int)meshes.size(); ++i)
    {
        if (!loaded[i]) continue;
        const Mesh& mesh = meshes[i];
        const BVH& bvh = bvhs[i];
        std::vector<char> visible(mesh.F.size(), 0);
        std::vector<Eigen::Vector3d> hitPoints;
        hitPoints.reserve(ndirs.size());

        // chunked processing（保持原有分块，便于缓存友好）
        const int N = (int)ndirs.size();
        const int CHUNK = 2048;

        for (int base = 0; base < N; base += CHUNK)
        {
            int end = std::min(N, base + CHUNK);
            std::vector<int> localHitTris; // 每个线程的局部命中三角形集合
            localHitTris.reserve(end - base);
            std::vector<Eigen::Vector3d> localHits;  // 每个线程的局部命中点集合
            localHits.reserve(end - base);

            for (int k = base; k < end; ++k)
            {
                double t;
                int ti;
                if (traverseBVHFirstHit(mesh, bvh, 0, cam_pos, ndirs[k], BACKFACE_CULL, t, ti))
                {
                    localHits.emplace_back(cam_pos + t * ndirs[k]);
                    if (ti >= 0)
                        localHitTris.push_back(ti);
                }
            }
            for (int ti : localHitTris)
                visible[ti] = 1;
            hitPoints.insert(hitPoints.end(), localHits.begin(), localHits.end());
        }

        // file-safe stem
        std::string stem = obj_files[i].filename().string();
        for (auto& c : stem)
            if (c == ' ' || c == ':' || c == '\\' || c == '/') c = '_';

        const std::string out_vis_faces = (out_dir / ("visible_faces_" + stem)).string();

        if (writeVisibleFacesOBJ(out_vis_faces + ".obj", mesh, visible))
        {
#pragma omp critical
            std::cout << "[OK] Faces -> " << out_vis_faces << ".obj\n";
        }
        const std::string out_vis_pts = (out_dir / ("visible_points_" + stem)).string();
        if (writePointsOBJ(out_vis_pts + ".obj", hitPoints)) {
#pragma omp critical
            std::cout << "[OK] Points -> " << out_vis_pts << ".obj\n";
        }
     
    }
    std::cout << "Done.\n";
    return 0;
}