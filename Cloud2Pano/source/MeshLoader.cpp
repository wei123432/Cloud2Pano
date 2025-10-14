#include "MeshLoader.h"
#include <iostream>
#include <GL/glew.h>
#include <tiny_obj_loader.h>


static GLMesh buildOne(const tinyobj::attrib_t& attrib,
   const std::vector<tinyobj::shape_t>& shapes)    //暂定shapes为每个瓦块的obj 
{
    GLMesh m; 
    std::vector<uint32_t> idx; //顶点索引数组
    std::vector<glm::vec3> pos; //顶点位置数组
    for (const auto& sh : shapes) {
        size_t off = 0; //记录偏移量 方便后续索引
        for (size_t f = 0; f < sh.mesh.num_face_vertices.size(); ++f) {  //mesh.num_face_vertices返回shape的每个面的顶点数量
            int fv = sh.mesh.num_face_vertices[f];//返回当前面的顶点数量
            if (fv != 3) throw std::runtime_error("Non-tri face");
            for (int v = 0; v < 3; ++v) {
                auto vi = sh.mesh.indices[off + v].vertex_index;//mesh.indices获取了索引数组。这里获取了顶点索引vertex_index
                glm::vec3 p(attrib.vertices[3 * vi + 0],
                            attrib.vertices[3 * vi + 1],
                            attrib.vertices[3 * vi + 2]);
                pos.push_back(p);
                idx.push_back((uint32_t)pos.size() - 1);
            }
            off += fv;
        }
 } 
    m.positions = pos;  
    m.indices = idx;

    glGenVertexArrays(1, &m.vao);
    glBindVertexArray(m.vao);

    glGenBuffers(1, &m.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, m.vbo);
    glBufferData(GL_ARRAY_BUFFER, pos.size() * sizeof(glm::vec3), pos.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);
   
    glGenBuffers(1, &m.ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, idx.size() * sizeof(uint32_t), idx.data(), GL_STATIC_DRAW);

    m.indexCount = (int)idx.size();
    glBindVertexArray(0);
    return m;
}

LoadedScene loadObjFolderBuildGL(const std::string& folder) {
    using std::filesystem::directory_iterator;
    LoadedScene S;
    uint32_t triOffset = 0;

    for (const auto& e : directory_iterator(folder)) {
        if (!e.is_regular_file()) continue;
        auto path = e.path().string();
        auto ext = e.path().extension().string();
        for (auto& c : ext) c = (char)tolower(c);
        if (ext != ".obj") continue;

        tinyobj::attrib_t attrib; 
        std::vector<tinyobj::shape_t> shapes; 
        std::vector<tinyobj::material_t> materials;
        std::string warn, err;
        bool ok = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, path.c_str(), nullptr, true, true);
        if (!ok) throw std::runtime_error("LoadObj failed: " + path + " " + warn + " " + err);

        GLMesh m = buildOne(attrib, shapes);
        m.baseTriIdOffset = triOffset;
        int triCount = m.indexCount / 3;
        S.meshes.push_back(m);

        for (int t = 0; t < triCount; ++t) S.idToMeshTri.push_back({ (int)S.meshes.size() - 1, t });
        triOffset += triCount;
    }
    return S;
}