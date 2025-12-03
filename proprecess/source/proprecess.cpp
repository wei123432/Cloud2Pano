#include <iostream>
#include <filesystem>
#include<fstream>
#include "Mesh.h"
#include"Loader.h"


int main()
{
	const  std::filesystem::path root_path ="D:\\experience\\Web\\data_origin\\3DModel";
    const auto  obj_path = root_path/"OBJ"/"Data";
    const auto  out_obj_path = root_path / "New_OBJ";
    std::vector<std::filesystem::path> obj_files = findAllOBJFiles(obj_path);
    std::filesystem::create_directories(out_obj_path);
    
#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < (int)obj_files.size(); ++i)
    {
        const auto& srcObj = obj_files[i];
        Mesh current_mesh;
        if (!loadOBJProcess(obj_files[i].string(), current_mesh))
        {
            std::cerr << "Failed to load (filtered) obj: " << srcObj << std::endl;
        }

        std::string tileName = srcObj.stem().string();
        const auto tile_dir = out_obj_path / tileName;
        std::filesystem::create_directories(tile_dir);
		const auto outObj = tile_dir / (tileName + ".obj");
        if (!saveOBJ(outObj.string(), current_mesh))
        {
            std::cerr << "Failed to save processed obj: " << outObj << std::endl;
		}

        std::cout << "Done. Filtered OBJs written to: " << out_obj_path << std::endl;
    }
    return 0;
}
