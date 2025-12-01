//逐文件读取
for (const auto& file : obj_files) {
    std::cout << "Found OBJ file: " << file << std::endl;
    if (loadOBJ(file.string(), mesh)) {
        std::cout << "Loaded OBJ file: " << file << " with "
            << mesh.V.size() << " vertices and "
            << mesh.F.size() << " faces." << std::endl;
    }
    else {
        std::cerr << "Failed to load OBJ file: " << file << std::endl;
    }


//未并行前的程序处理
        std::vector<Eigen::Vector3d> hitPoints;
        hitPoints.reserve(directions.size());
        std::vector<char> visible(imesh.F.size(), 0); 
		std::vector<int> hitTriIdx; //击中的三角形索引

        for (const auto& ndir : directions) {
            double t;
            int ti;
            if (traverseBVHFirstHit(imesh, bvh, 0, cam_pos, ndir, BACKFACE_CULL, t, ti)) 
            {
                hitPoints.emplace_back(cam_pos + t * ndir); //返回了击中的点
                if (ti >= 0) 
                    visible[ti] = 1; //得到击中的面
            }
        }
        
        for (size_t i = 0; i < visible.size(); i++)
        {
            if (visible[i] == 1)
                hitTriIdx.push_back((int)i);
        }

        for (size_t j = 0; j < hitTriIdx.size(); j++)
        {
			std::cout << "Hit triangle index: " << hitTriIdx[j] << std::endl; 
        }

        //根据击中三角形得到可视模型
        static int outIdx = 0;
        
        const std::string output_path =
            (out_dir / ("visible_" + std::to_string(outIdx++) + ".obj")).string();

        if (writeVisibleFacesOBJ(output_path, imesh, visible))
            std::cout << "Wrote visible-face subset OBJ: " << output_path << "\n";
        else
            std::cout << "Failed to write: " << output_path << "\n";
    }



//单瓦块组成单模型寻找可视面与交点
const auto out_dir_faces = std::filesystem::path("D:\\experience\\try\\Visualmodel\\face_select-true");
const auto out_dir_points = std::filesystem::path("D:\\experience\\try\\Visualmodel\\point_hit-true");
std::filesystem::create_directories(out_dir_faces);
std::filesystem::create_directories(out_dir_points);

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
        std::vector<Eigen::Vector3d> localHits; // 每个线程的局部命中点集合
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
    std::string stem = obj_files[i].filename().string();
    const std::string out_vis_faces = (out_dir_faces / ("visible_faces_" + stem)).string();
    if (writeVisibleFacesOBJ(out_vis_faces, mesh, visible))
    {
#pragma omp critical
        std::cout << "[OK] Faces -> " << out_vis_faces << ".obj\n";
    }
    const std::string out_vis_pts = (out_dir_points / ("visible_points_" + stem)).string();
    if (writePointsOBJ(out_vis_pts, hitPoints)) {
#pragma omp critical
        std::cout << "[OK] Points -> " << out_vis_pts << ".obj\n";
    }
}

//读取位姿文件
std::cout << "成功读取 " << translations.size() << " 行数据" << std::endl;

// 打印读取结果（示例）
for (size_t i = 0; i < translations.size(); i++) 
{
    std::cout << "第 " << i + 1 << " 行:" << std::endl;
    std::cout << "  平移向量: ";
    for (double val : translations[i]) 
    {
        std::cout << val << " ";
    }
    std::cout << std::endl;

    std::cout << "  四元数: ";
    for (double val : quaternions[i]) 
    {
        std::cout << val << " ";
    }
    std::cout << std::endl << std::endl;
}




// 根据面与面之间是否共享顶点，把 Mesh 拆成多个连通块
void computeFaceComponents(
    const Mesh& m,
    std::vector<int>& faceComp,  // 输出：每个面的组件 id
    int& numComponents)          // 输出：组件数量
{
    const int F = static_cast<int>(m.F.size());
    faceComp.assign(F, -1);
    numComponents = 0;

    // 顶点 -> 相邻面列表
    std::vector<std::vector<int>> vertexFaces(m.V.size());
    for (int fi = 0; fi < F; ++fi)
    {
        const auto& f = m.F[fi];
        vertexFaces[f.i0].push_back(fi);
        vertexFaces[f.i1].push_back(fi);
        vertexFaces[f.i2].push_back(fi);
    }

    std::vector<int> stack;
    stack.reserve(64);

    for (int fi = 0; fi < F; ++fi)
    {
        if (faceComp[fi] != -1) continue; // 已经标过

        int cid = numComponents++;
        faceComp[fi] = cid;
        stack.clear();
        stack.push_back(fi);

        // DFS / BFS 都行，这里用 stack 做 DFS
        while (!stack.empty())
        {
            int cur = stack.back();
            stack.pop_back();

            const auto& cf = m.F[cur];
            int vids[3] = { cf.i0, cf.i1, cf.i2 };

            for (int k = 0; k < 3; ++k)
            {
                int v = vids[k];
                for (int nbFace : vertexFaces[v])
                {
                    if (faceComp[nbFace] == -1)
                    {
                        faceComp[nbFace] = cid;
                        stack.push_back(nbFace);
                    }
                }
            }
        }
    }
}

//给每一个联通块计算AABB
void buildComponentBoxes(
    const Mesh& m,
    const std::vector<int>& faceComp,
    int numComponents,
    double padding,
    std::vector<AABB>& compBoxes)
{
    compBoxes.assign(numComponents, AABB());

    // 用每个面的三个顶点扩对应组件的 AABB
    for (int fi = 0; fi < static_cast<int>(m.F.size()); ++fi)
    {
        int cid = faceComp[fi];
        auto& box = compBoxes[cid];

        const auto& f = m.F[fi];
        box.expand(m.V[f.i0]);
        box.expand(m.V[f.i1]);
        box.expand(m.V[f.i2]);
    }

    // 在每个 AABB 外面扩一圈 padding
    Eigen::Vector3d pad(padding, padding, padding);
    for (int c = 0; c < numComponents; ++c)
    {
        compBoxes[c].bmin -= pad;
        compBoxes[c].bmax += pad;
    }
}