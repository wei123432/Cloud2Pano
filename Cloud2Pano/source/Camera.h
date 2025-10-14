#pragma once

#define _USE_MATH_DEFINES
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>

class Camera
{
public:
	Camera() {};
	Camera(const Eigen::Vector3d& position, const Eigen::Quaterniond& quaternion)
		:c_position(position),c_quaternion(quaternion)
	{
		pose = CameraToWorld();
	};
	Eigen::Matrix4d pose; //相机位姿
private:
	Eigen::Vector3d c_position; //相机世界坐标系的位置
	Eigen::Quaterniond c_quaternion; //相机初始四元数

public:
    struct GridRegion 
    {
        Eigen::Vector3d dir; // 加密方向
        double sigma_deg;    // 影响半角（角度）
        double strength;      // 强度
    };

	//生成球面均匀采样方向
    std::vector<Eigen::Vector3d> GenerateSphereGridDirections() const
    {
        constexpr uint64_t LON_TARGETNum = 10000; 
        constexpr uint64_t LAT_TARGETNum = 5000;
        constexpr uint64_t MAX_POINTS = 1e8; 
        const long double total = static_cast<long double>(LON_TARGETNum) * static_cast<long double>(LAT_TARGETNum);
        // 计算抽稀步长
        uint64_t lon_step = 1, lat_step = 1;
        if (total > static_cast<long double>(MAX_POINTS)) 
        {
            //纬度抽稀
            long double r = total / static_cast<long double>(MAX_POINTS); 
            long double s = std::sqrt(r);
			lat_step = static_cast<uint64_t>(std::ceil(s)); //向上取整得到纬度步长
            if (lat_step == 0) 
                lat_step = 1;
            // 经向抽稀
            long double r2 = r / static_cast<long double>(lat_step);
            lon_step = static_cast<uint64_t>(std::ceil(std::max< long double >(1.0, r2)));
        }
        // 实际使用的经/纬采样数
        const uint64_t N_LON = (LON_TARGETNum + lon_step - 1) / lon_step; 
        const uint64_t N_LAT = (LAT_TARGETNum + lat_step - 1) / lat_step;
        std::vector<Eigen::Vector3d> dirs;
        dirs.reserve(static_cast<size_t>(std::min<uint64_t>(N_LON * N_LAT, MAX_POINTS)));
        const long double d_lambda = 2* M_PI / static_cast<long double>(LON_TARGETNum);
        const long double d_phi = M_PI / static_cast<long double>(LAT_TARGETNum);
        for (uint64_t ilat = 0; ilat < LAT_TARGETNum; ilat +=lat_step) 
        {
            long double f_phi = (static_cast<long double>(ilat) + 0.5) * d_phi; 
            long double phi = f_phi - M_PI_2; // 转换为地理纬度
            long double sinφ = std::sin(phi);
            long double cosφ = std::cos(phi);
            for (uint64_t ilon = 0; ilon < LAT_TARGETNum; ilon += lon_step) 
            {
                long double lambda = (static_cast<long double>(ilon) + 0.5) * d_lambda; 
                long double cosλ = std::cos(lambda);
                long double sinλ = std::sin(lambda);
                // 单位球方向（相机为球心，R=1）
                Eigen::Vector3d v(
                    static_cast<double>(cosφ * cosλ),
                    static_cast<double>(sinφ),
                    static_cast<double>(cosφ * sinλ)
                );
                dirs.emplace_back(v);
            }
        }
        return dirs;
    }

    // 返回“世界坐标”的网格点位置
    std::vector<Eigen::Vector3d> GenerateSphereGridPointsWorld() const
    {
        auto dirs = GenerateSphereGridDirections();
        std::vector<Eigen::Vector3d> pts;
        pts.reserve(dirs.size());
        for (const auto& d : dirs) 
            pts.emplace_back(c_position + d);
        return pts;
    }

    Eigen::Matrix4d CameraToWorld()
    {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();//初始化单位阵
        Eigen::Matrix3d rotationMatrix = c_quaternion.toRotationMatrix();//四元数获得旋转矩阵
        transform.block<3, 3>(0, 0) = rotationMatrix;
        transform.block<3, 1>(0, 3) = c_position;
        return transform;  //相机系到世界系的变换矩阵
    }
};