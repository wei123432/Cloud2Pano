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

//模型的包围盒计算
static AABB computeMeshBox(const Mesh& m)
{
    AABB box;
    for (const auto& v : m.V)
        box.expand(v);
    return box;
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

//AABB包围盒内部判断
static bool overlaps(const AABB& a, const AABB& b)
{
    if (a.bmax.x() < b.bmin.x() || a.bmin.x() > b.bmax.x()) return false;
    if (a.bmax.y() < b.bmin.y() || a.bmin.y() > b.bmax.y()) return false;
    if (a.bmax.z() < b.bmin.z() || a.bmin.z() > b.bmax.z()) return false;
    return true;
}

// 点是否在 AABB 内（含边界）
static bool contains(const AABB& box, const Eigen::Vector3d& p)
{
    return (p.x() >= box.bmin.x() && p.x() <= box.bmax.x() &&
        p.y() >= box.bmin.y() && p.y() <= box.bmax.y() &&
        p.z() >= box.bmin.z() && p.z() <= box.bmax.z());
}

//用可视模型包围盒裁切模型
static void queryBVH_AABB(
    const BVH& bvh,
    int nodeIdx,
    const AABB& queryBox,
    std::vector<int>& outTriIndices)
{
    if (nodeIdx < 0) return;
    const BVHNode& node = bvh.nodes[nodeIdx];

    // 节点包围盒与查询盒不相交，直接返回
    if (!overlaps(node.box, queryBox))
        return;

    // 叶子节点
    if (node.count > 0)
    {
        for (int i = 0; i < node.count; ++i)
        {
            int triIndex = bvh.triIdx[node.start + i]; // triIdx 映射到 mesh->F 下标
            outTriIndices.push_back(triIndex);
        }
    }
    else
    {
        // 内部节点：递归左右子树
        queryBVH_AABB(bvh, node.left, queryBox, outTriIndices);
        queryBVH_AABB(bvh, node.right, queryBox, outTriIndices);
    }
}

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
    double& io_tmax, bool backfaceCull)   // ti是每个面的索引 o是原点的位置向量 d是方向向量 backfaceCull是背面剔除参数 io_max是当前已知的有效交点
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

//2D占用网格结构
struct OccupancyGrid2D
{
    int nx = 0, ny = 0; //网格行列数
	double xmin = 0, ymin = 0; //网格起始坐标
    double dx = 1, dy = 1; // 网格边长
    std::vector<unsigned char> data; // 网格占用状态

    //初始化
    void init(double xmin_, double xmax_, double ymin_, double ymax_, int nx_, int ny_)
    {
        nx = nx_;
        ny = ny_;
        xmin = xmin_;
        ymin = ymin_;
        dx = (xmax_ - xmin_) / std::max(1, nx - 1);
        dy = (ymax_ - ymin_) / std::max(1, ny - 1);
        data.assign(nx * ny, 0);
    }

	// 判断索引是否在网格内
    inline bool insideIndex(int ix, int iy) const
    {
        return (ix >= 0 && ix < nx && iy >= 0 && iy < ny);
    }

	// 世界坐标转网格索引
    inline void worldToIndex(double x, double y, int& ix, int& iy) const
    {
        ix = (int)std::floor((x - xmin) / dx + 0.5);
        iy = (int)std::floor((y - ymin) / dy + 0.5);
    }

	//标记占用
    void mark(double x, double y)
    {
        int ix, iy;
        worldToIndex(x, y, ix, iy);
        if (insideIndex(ix, iy))
            data[iy * nx + ix] = 1;
    }

	//判断网格索引是否被占用
    bool isOccupied(double x, double y) const
    {
        int ix, iy;
        worldToIndex(x, y, ix, iy);
        if (!insideIndex(ix, iy))
            return false;
        return data[iy * nx + ix] != 0;
    }

    // 放大占用区域
    void dilate(int iterations)
    {
        for (int it = 0; it < iterations; ++it)
        {
            std::vector<unsigned char> tmp = data;
            for (int iy = 0; iy < ny; ++iy)
            {
                for (int ix = 0; ix < nx; ++ix)
                {
                    if (data[iy * nx + ix]) continue;
                    bool hit = false;
                    for (int dy_ = -1; dy_ <= 1 && !hit; ++dy_)
                        for (int dx_ = -1; dx_ <= 1 && !hit; ++dx_)
                        {
                            int jx = ix + dx_, jy = iy + dy_;
                            if (insideIndex(jx, jy) && data[jy * nx + jx])
                                hit = true;
                        }
                    if (hit) tmp[iy * nx + ix] = 1;
                }
            }
            data.swap(tmp);
        }
    }

    // 缩小占用区域
    void erode(int iterations)
    {
        for (int it = 0; it < iterations; ++it)
        {
            std::vector<unsigned char> tmp = data;
            for (int iy = 0; iy < ny; ++iy)
            {
                for (int ix = 0; ix < nx; ++ix)
                {
                    if (!data[iy * nx + ix]) continue;
                    bool allNeighbors = true;
                    for (int dy_ = -1; dy_ <= 1 && allNeighbors; ++dy_)
                        for (int dx_ = -1; dx_ <= 1 && allNeighbors; ++dx_)
                        {
                            int jx = ix + dx_, jy = iy + dy_;
                            if (!insideIndex(jx, jy) || !data[jy * nx + jx])
                                allNeighbors = false;
                        }
                    if (!allNeighbors) tmp[iy * nx + ix] = 0;
                }
            }
            data.swap(tmp);
        }
    }
};

// 根据可视模型构建 XY 平面的占用掩码，并返回 Z 范围
void buildVisibleMaskXY
    (const Mesh& visibleMesh,
     OccupancyGrid2D& grid,
     double& zMin, double& zMax)
{
    double xmin = std::numeric_limits<double>::infinity();
    double xmax = -std::numeric_limits<double>::infinity();
    double ymin = std::numeric_limits<double>::infinity();
    double ymax = -std::numeric_limits<double>::infinity();
    zMin = std::numeric_limits<double>::infinity();
    zMax = -std::numeric_limits<double>::infinity();

    for (const auto& v : visibleMesh.V)
    {
        xmin = std::min(xmin, v.x());
        xmax = std::max(xmax, v.x());
        ymin = std::min(ymin, v.y());
        ymax = std::max(ymax, v.y());
        zMin = std::min(zMin, v.z());
        zMax = std::max(zMax, v.z());
    }

    double padXY = 0.5; 
    xmin -= padXY; xmax += padXY;
    ymin -= padXY; ymax += padXY;

    //按范围初始化网格（分辨率可以根据场景调）
    const int nx = 1024;
    const int ny = 1024;
    grid.init(xmin, xmax, ymin, ymax, nx, ny);

    //把可视模型的顶点投影到 XY，标记为占用
    for (const auto& v : visibleMesh.V)
    {
        grid.mark(v.x(), v.y());
    }

    //做一次形态学“闭操作”：先膨胀再腐蚀，填小洞、连细缝
    int iter = 2; 
    grid.dilate(iter);
    grid.erode(iter);

    //再额外膨胀一圈
    double extraXY = 0.3; 
    double cellSize = std::max(grid.dx, grid.dy);
    int extraIter = std::max(1, (int)std::ceil(extraXY / cellSize));

    grid.dilate(extraIter);
}
