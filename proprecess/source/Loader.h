#pragma once
#define _USE_MATH_DEFINES
#define X_SRS -60.590000
#define Y_SRS -19.490000
#define Z_SRS -6.190000
#define Z_filter 4.200000
#include<iostream>
#include <sstream>
#include<fstream>
#include <filesystem>
#include<cmath>
#include"Mesh.h"

//读取相机位姿数据
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
    while (std::getline(file, s)) {
        std::istringstream iss(s);
        std::vector<double> data;
        std::string token;

        // 按空格分割每行数据
        while (iss >> token) {
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

// 把obj的索引改为从0开始
static int parseIndex(const std::string& indice_string)
{
    size_t slash = indice_string.find('/');
    std::string s = (slash == std::string::npos) ? indice_string : indice_string.substr(0, slash);
    int idx = std::stoi(s);
    return idx - 1;
}
//加载obj数据前剔除数据
static bool loadOBJProcess(const std::string& path, Mesh& mesh)
{
    std::ifstream ifs(path);
    if (!ifs) return false;
    mesh.V.clear();
    mesh.F.clear();

    std::string line;
    std::vector<int> face;
    std::vector<int> oldToNew;

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

            int newIndex = -1;
			double Z = z + Z_SRS;
            if ( Z < Z_filter)
            {
                newIndex = (int)mesh.V.size();
                mesh.V.emplace_back(x, y, z);
            }
            oldToNew.push_back(newIndex);
        }
    }
	// 重新读取文件，处理面信息
        ifs.clear();
        ifs.seekg(0, std::ios::beg);

        while (std::getline(ifs, line))
        { 
            if (line.empty() || line[0] == '#') continue;
            std::istringstream ss(line);
            std::string tag;
            ss >> tag;

            if (tag == "f")
            {
                face.clear();
                std::string token;
                bool discardFace = false;

                while (ss >> token)
                {
                    try
                    {
                        int oldIdx = parseIndex(token);
                        // 越界保护
                        if (oldIdx < 0 || oldIdx >= static_cast<int>(oldToNew.size()))
                        {
                            discardFace = true;
                            break;
                        }

                        int newIdx = oldToNew[oldIdx];
                        if (newIdx == -1)
                        {
                            discardFace = true;
                            break;
                        }

                        face.push_back(newIdx);
                    }
                    catch (...) 
                    {
                        discardFace = true;
                        break;
                    }
                }

                if (discardFace) continue;

                if (face.size() >= 3)
                {
                    for (size_t j = 1; j + 1 < face.size(); ++j)
                    {
                        mesh.F.push_back({ face[0], face[j], face[j + 1] });
                    }
                }
            }
        }
    return true;
}

// 加载obj文件
static bool loadOBJ(const std::string& path, Mesh& mesh) 
{
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
        else if (tag == "f")
        {
            face.clear();
            std::string token;
            while (ss >> token)
            {
                try
                {
                    face.push_back(parseIndex(token));
                }
                catch (...) {}
            }
            if (face.size() >= 3)
            {
                for (size_t j = 1; j + 1 < face.size(); ++j)
                {
                    mesh.F.push_back({ face[0], face[j], face[j + 1] });
                }
            }
        }
    }
    return true;
}

//加载后续数据obj
static bool loadOBJNoSRS(const std::string& path, Mesh& mesh)
{
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
        else if (tag == "f")
        {
            face.clear();
            std::string token;
            while (ss >> token)
            {
                try
                {
                    face.push_back(parseIndex(token));
                }
                catch (...) {}
            }
            if (face.size() >= 3)
            {
                for (size_t j = 1; j + 1 < face.size(); ++j)
                {
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

//保存obj文件
bool saveOBJ(const std::string& filename, const Mesh& mesh)
{
    std::ofstream out(filename);
    if (!out.is_open())
    {
        std::cerr << "Failed to open OBJ file for writing: "
            << filename << std::endl;
        return false;
    }

    // 写顶点（v x y z）
    // 使用固定小数格式，精度你可以自己调整
    out << std::fixed << std::setprecision(6);

    for (const auto& v : mesh.V)
    {
        out << "v "
            << v.x() << " "
            << v.y() << " "
            << v.z() << "\n";
    }

    // 写面（f i j k）注意：OBJ 的索引是 **从 1 开始** 的
    for (const auto& f : mesh.F)
    {
        // 如果你没有纹理/法线，就直接写顶点索引
        out << "f "
            << (f.i0 + 1) << " "
            << (f.i1 + 1) << " "
            << (f.i2 + 1) << "\n";
    }

    out.close();
    return true;
}