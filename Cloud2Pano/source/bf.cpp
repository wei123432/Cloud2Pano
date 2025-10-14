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