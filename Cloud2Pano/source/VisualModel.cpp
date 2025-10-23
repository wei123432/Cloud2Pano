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
#include "Pano.h"
#include "Algorithm.h"
#include "Mesh.h"

//读取相机的数据文件
bool readData(const std::string& filename,
    std::vector<std::vector<double>>& translations,
    std::vector<std::vector<double>>& quaternions)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "无法打开文件: " << filename << std::endl;
        return false;
    }

    std::string s;
    // 逐行读取文件
    while (std::getline(file, s)) 
    {
        std::istringstream iss(s);
        std::vector<double> data;
        std::string token;

        // 按空格分割每行数据
        while (iss >> token) 
        {
            // 尝试将字符串转换为数字
            double value = std::stod(token);
            data.push_back(value);
        }

        // 检查数据是否足够（至少需要9个数字）
        if (data.size() >= 7) {
            // 提取第三、四、五个数据（索引2、3、4）
            std::vector<double> trans = { data[2], data[3], data[4] };
            // 提取第六到第九个数据（索引5、6、7、8）
            std::vector<double> quat = { data[5], data[6], data[7], data[8] };

            translations.push_back(trans);
            quaternions.push_back(quat);
        }
        else {
            std::cout << "数据格式不正确，跳过该行: " << s << std::endl;
        }
    }

    file.close();
    return true;
}


//辅助循环体
struct RayHit {
    double t = std::numeric_limits<double>::infinity();
    int mesh_idx = -1;
    int tri_idx = -1;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    bool hit = false;
};


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
            x += X_SRS;
			y += Y_SRS;
			z += Z_SRS;
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
static bool writeVisibleFacesOBJ(const std::string& path, const Mesh& m, const std::vector<char>& visible) 
{
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
    // Camera位姿
    /*const Eigen::Vector3d cam_pos(-0.22033, -0.106948, 1.532681);*/
    /*const Eigen::Vector3d cam_pos(-0.507947,- 0.162965,1.523007);*/
    /*const Eigen::Vector3d cam_pos(-6.051642,- 0.567130,1.574263);*/
    const Eigen::Vector3d cam_pos(-39.290264,4.874366,1.596728);
    const Eigen::Quaterniond cam_quat(-0.122489, -0.679658, -0.135731, 0.710379);
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

    const std::filesystem::path obj_path = "D:\\experience\\try\\DasModel\\3DModel\\OBJ\\Data";
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
    auto start = std::chrono::high_resolution_clock::now();
    //多瓦快组成单模型寻找可视面与交点
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

    const auto out_root = std::filesystem::path("D:\\experience\\try\\Visualmodel\\merged5");
    std::filesystem::create_directories(out_root);

    const std::string out_faces = (out_root / "visible_all.obj").string();
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