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