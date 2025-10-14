#pragma once
#define EPS 1e-12
#define TMIN 1e-6
#include <Eigen/Dense>
#include <vector>
#include "Mesh.h"

//AABB包围盒
struct AABB 
{
    Eigen::Vector3d bmin{ std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::infinity() };
    Eigen::Vector3d bmax{ -std::numeric_limits<double>::infinity(),
                         -std::numeric_limits<double>::infinity(),
						 -std::numeric_limits<double>::infinity() }; //初始化（-∞,+∞）
    void expand(const Eigen::Vector3d& p) 
    {
        bmin = bmin.cwiseMin(p);
        bmax = bmax.cwiseMax(p);
	}  //用点扩展包围盒
    void expand(const AABB& b) 
    {
        bmin = bmin.cwiseMin(b.bmin);
        bmax = bmax.cwiseMax(b.bmax);
	}  //合并两个包围盒
};

//三角形的包围盒和重心
static AABB triBounds(const Mesh& m, int ti) 
{
    const auto& v0 = m.V[m.F[ti].i0], & v1 = m.V[m.F[ti].i1], & v2 = m.V[m.F[ti].i2];
    AABB b; 
    b.expand(v0); 
    b.expand(v1); 
    b.expand(v2); 
    return b;
}
static Eigen::Vector3d triCentroid(const Mesh& m, int ti) 
{
    const auto& v0 = m.V[m.F[ti].i0], & v1 = m.V[m.F[ti].i1], & v2 = m.V[m.F[ti].i2];
    return (v0 + v1 + v2) / 3.0;
}

//BVH节点
struct BVHNode {
    AABB box; //包围盒数据
	int left = -1, right = -1; // 内部节点
    int start = 0, count = 0; // 叶子区间
};

// BVH层次结构
struct BVH {
    const Mesh* mesh = nullptr;  
    std::vector<int> triIdx;        // 三角形索引列表
    std::vector<BVHNode> nodes;     // BVH节点列表

    // 递归构建BVH
    int buildRec(int start, int end) 
    {
        //计算所有三角形的包围盒
        BVHNode node;
        AABB b;
        for (int i = start; i < end; ++i) 
        {
            b.expand(triBounds(*mesh, triIdx[i]));
        }
        node.box = b; 
		//如果三角形数量小于等于8，则创建叶子节点
        int n = end - start;
        if (n <= 8) 
        {  
            node.start = start;  
            node.count = n;      
            nodes.push_back(node);
            return (int)nodes.size() - 1; 
        }
        //创建左右子树
        AABB cb;
        for (int i = start; i < end; ++i)
        {
            cb.expand(triCentroid(*mesh, triIdx[i]));
        }
        //设置划分轴
		Eigen::Vector3d ext = cb.bmax - cb.bmin;  
        int axis = 0;
        if (ext.y() >= ext.x() && ext.y() >= ext.z()) 
            axis = 1;
        else if (ext.z() >= ext.x() && ext.z() >= ext.y()) 
            axis = 2; 

        int mid = (start + end) / 2;
        // 部分排序：使mid位置左边的重心在axis上的值 <= 右边
        std::nth_element(
            triIdx.begin() + start, 
            triIdx.begin() + mid, 
            triIdx.begin() + end,
            [&](int a, int b) 
                {  
                    return triCentroid(*mesh, a)[axis] < triCentroid(*mesh, b)[axis]; 
                });

        int id = (int)nodes.size();
        nodes.push_back(BVHNode());   //处理非叶节点时提前占位
        int L = buildRec(start, mid);   
        int R = buildRec(mid, end);
        nodes[id] = node;
        nodes[id].left = L;
        nodes[id].right = R;
        return id;  // 返回当前节点索引
    }

	//初始化模型的索引并构建BVH
    void build(const Mesh* m) 
    {
        mesh = m;
        triIdx.resize(mesh->F.size());
        for (size_t i = 0; i < mesh->F.size(); ++i) 
            triIdx[i] = (int)i;
        nodes.clear();
        nodes.reserve(mesh->F.size() * 2);
        buildRec(0, (int)triIdx.size());
    }
};

// 光线与AABB盒相交测试
static inline bool rayAABB(const Eigen::Vector3d& o, const Eigen::Vector3d& d, 
    const Eigen::Vector3d& invd,const AABB& box) 
{
    Eigen::Vector3d t1 = (box.bmin - o).cwiseProduct(invd);
    Eigen::Vector3d t2 = (box.bmax - o).cwiseProduct(invd);
    double tmin = std::max({ std::min(t1.x(), t2.x()), std::min(t1.y(), t2.y()), std::min(t1.z(), t2.z()) });
    double tmax = std::min({ std::max(t1.x(), t2.x()), std::max(t1.y(), t2.y()), std::max(t1.z(), t2.z()) });
    return tmax >= std::max(tmin, 0.0);
}

// Möller–Trumbore算法 
static bool rayTriFirstHit(const Mesh& m, int ti, const Eigen::Vector3d& o, const Eigen::Vector3d& d,
    double& io_tmax, bool backfaceCull)   // ti是每个面三个顶点的索引 o是原点的位置向量 d是方向向量 backfaceCull是背面剔除参数 io_max是当前已知的有效交点
{
    const auto& v0 = m.V[m.F[ti].i0], & v1 = m.V[m.F[ti].i1], & v2 = m.V[m.F[ti].i2];
    Eigen::Vector3d e1 = v1 - v0;
    Eigen::Vector3d e2 = v2 - v0;
    Eigen::Vector3d pvec = d.cross(e2);
    double det = e1.dot(pvec);  //方程组系数矩阵的行列式(线性方程组的系数）
    if (backfaceCull) 
    {
        if (det <= EPS) 
            return false;
        Eigen::Vector3d tvec = o - v0;
        double u = tvec.dot(pvec); //参数方程的系数u
        if (u < 0.0 || u > det) 
            return false;
        Eigen::Vector3d qvec = tvec.cross(e1);//参数方程的系数v
        double v = d.dot(qvec);
        if (v < 0.0 || u + v > det) 
            return false;
        double invDet = 1.0 / det;
        double t = e2.dot(qvec) * invDet; //方程参数t 表示交点到起点的距离
        if (t > TMIN && t < io_tmax) 
        { 
            io_tmax = t; 
            return true; 
        }
        return false;
    }
    else 
    {
        if (std::abs(det) < EPS) 
            return false;
        double invDet = 1.0 / det;
        Eigen::Vector3d tvec = o - v0;
        double u = tvec.dot(pvec) * invDet;
        if (u < 0.0 || u > 1.0) 
            return false;
        Eigen::Vector3d qvec = tvec.cross(e1);
        double v = d.dot(qvec) * invDet;
        if (v < 0.0 || u + v > 1.0) 
            return false;
        double t = e2.dot(qvec) * invDet;
        if (t > TMIN && t < io_tmax) 
        { 
            io_tmax = t; 
            return true; 
        }
        return false;
    }
}

//基于BVH进行交点检测
static bool traverseBVHFirstHit(const Mesh& m, const BVH& bvh, int root,
    const Eigen::Vector3d& o, const Eigen::Vector3d& d,
    bool backfaceCull, double& out_t, int& out_ti) 
{
    Eigen::Vector3d invd(1.0 / (std::abs(d.x()) < EPS ? std::copysign(EPS, d.x()) : d.x()),
        1.0 / (std::abs(d.y()) < EPS ? std::copysign(EPS, d.y()) : d.y()),
        1.0 / (std::abs(d.z()) < EPS ? std::copysign(EPS, d.z()) : d.z()));
    std::vector<int> stack; 
    stack.reserve(128); 
    stack.push_back(root);
    double tmax = std::numeric_limits<double>::infinity();
    int hit = -1;

    while (!stack.empty()) {
		int ni = stack.back(); //获取栈顶节点
		stack.pop_back();      //移除栈顶节点
        const auto& node = bvh.nodes[ni];
        if (!rayAABB(o, d, invd, node.box)) continue; //slab算法先判断是否穿过包围盒
        if (node.left == -1 && node.count > 0) 
        {
            for (int i = 0; i < node.count; ++i) 
            {
                int ti = bvh.triIdx[node.start + i];
                double t = tmax;
                if (rayTriFirstHit(m, ti, o, d, t, backfaceCull)) 
                {
                    if (t < tmax) 
                        { 
                          tmax = t; 
                          hit = ti; 
                        }
                }
            }
        }
        else {
            stack.push_back(node.left);
            stack.push_back(node.right);
        }
    }
    if (hit >= 0) 
    { 
        out_t = tmax; 
        out_ti = hit;   //返回击中的最近的三角形
        return true; 
    }
    return false;
}