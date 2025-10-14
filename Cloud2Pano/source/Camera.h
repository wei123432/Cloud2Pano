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

  //  // 生成单位球面网格方向（中心=相机位置，R=1）
  //  std::vector<Eigen::Vector3d> GenerateSphereGridDirections(
  //      double s,                                       // 目标弧长 s（R=1即弧度）
  //      const std::vector<GridRegion>& regions = {},    // 不同大小网格加密区
  //      int   min_longitude = 20,                       // 每条纬线最少经度点数（>=3）
  //      bool  include_poles = true                      // 是否包含两极点
  //  ) const
  //  {
  //      std::vector<Eigen::Vector3d> dirs; //返回的方向向量
  //      if (s <= 0.0) 
  //          return dirs; 

  //      auto clampd = [&](double value, double max_number, double min_number)
  //          { 
  //              return std::max(min_number, std::min(value, min_number)); 
		//	};  //返回value在min_number和max_number之间的值

		//// 方向向量 -> y=sin(φ)
  //      auto dir_to_lat = [&](const Eigen::Vector3d& va)
  //          {
  //          double y = std::clamp(va.y(), -1.0, 1.0);
  //          return std::asin(y);
  //          };
  //  
		//// 高斯权重 w(φ)
  //      auto weight_lat = [&](double lat)
  //          {
  //          double w = 1.0;
  //          for (const auto& r : regions) {
  //              Eigen::Vector3d dir_normal = r.dir.normalized();
  //              double lat_k = dir_to_lat(dir_normal);
  //              double sigma = r.sigma_deg * M_PI / 180.0; 
  //              if (sigma <= 1e-9) continue;
  //              double d = (lat - lat_k) / sigma;
  //              w += r.strength * std::exp(-0.5 * d * d);
  //          }
  //          return std::max(1.0, w);
  //      };

		//// 生成纬度列表（近等弧长）：Δφ ≈ s / w(φ)（R=1）
  //      std::vector<double> lat_list;
  //      double lat = -M_PI/2.0;
  //      if (include_poles) lat_list.push_back(lat);

  //      while ( lat < M_PI/2.0 - 1e-9) {
  //          double w  = weight_lat(lat);
  //          double dφ = clampd(s / w, 1e-5, 0.2);  //弧度制
  //          double next_lat = lat + dφ;
  //          if (next_lat > M_PI /2.0) 
  //                  next_lat = M_PI /2.0;
  //          if (lat_list.empty() || std::abs(next_lat - lat_list.back()) > 5e-4)
  //                  lat_list.push_back(next_lat);
  //          lat = next_lat;
  //      }

  //      // 每条纬线上根据 cos(φ) 自适应经度采样，使经向弧长接近 s/w(φ)
  //      for (double φ : lat_list) {
  //          double cosφ = std::cos(φ);
  //          double sinφ = std::sin(φ);

  //          // 极点
  //          if (std::abs(cosφ) < 1e-10) 
  //          {
  //              if (include_poles) dirs.emplace_back(0.0, sinφ, 0.0); 
  //              continue;
  //          }

  //          double weight = weight_lat(φ);
  //          double s_w  = s / weight;
  //          int n_lon = (int)std::llround( (2.0 * M_PI * std::abs(cosφ)) / std::max(1e-12, s_w) );
  //          n_lon = std::max(n_lon, min_longitude);

  //          for (int j = 0; j < n_lon; ++j) {
  //              double λ = (2.0 * M_PI * j) / n_lon; // 经度取值[0, 2π)
  //              double cosλ = std::cos(λ);
  //              double sinλ = std::sin(λ);
  //              Eigen::Vector3d v(cosφ * cosλ, sinφ, cosφ * sinλ);
  //              dirs.emplace_back(v); 
  //          }
  //      }
  //      return dirs;
  //  }
    std::vector<Eigen::Vector3d> GenerateSphereGridDirections(
        double /*s_ignored*/,                               // 保留形参以不改接口，但此版本不使用
        const std::vector<GridRegion> & /*regions_ignored*/ = {},
        int   /*min_longitude_ignored*/ = 20,
        bool  /*include_poles_ignored*/ = true
    ) const
    {
        // 目标格子数（按需求）
        constexpr uint64_t N_LON_TARGET = 10000; // 经度分段数（0..360°）
        constexpr uint64_t N_LAT_TARGET = 5000; // 纬度分段数（0..180°）

        // 安全上限：最多生成这么多方向，超出则自动抽稀（可按机器内存调大/调小）
        constexpr uint64_t MAX_POINTS_CAP = 10'000'000ULL; // 约 1e7 个方向（~240 MB 内存）

        const long double total = static_cast<long double>(N_LON_TARGET) * static_cast<long double>(N_LAT_TARGET);

        // 计算抽稀步长
        uint64_t lon_step = 1, lat_step = 1;
        if (total > static_cast<long double>(MAX_POINTS_CAP)) {
            // 需要抽稀：把总缩减比 r 分摊到经/纬两个方向，尽量接近平方根均分
            long double r = total / static_cast<long double>(MAX_POINTS_CAP); // 需要缩减的倍率
            long double s = std::sqrt(r);
            lat_step = static_cast<uint64_t>(std::ceil(s));
            if (lat_step == 0) lat_step = 1;
            // 经向再按剩余比例取整
            long double r2 = r / static_cast<long double>(lat_step);
            lon_step = static_cast<uint64_t>(std::ceil(std::max< long double >(1.0L, r2)));
        }

        // 实际使用的经/纬采样数
        const uint64_t N_LON = (N_LON_TARGET + lon_step - 1) / lon_step; // 向上取整后的采样次数
        const uint64_t N_LAT = (N_LAT_TARGET + lat_step - 1) / lat_step;

        std::vector<Eigen::Vector3d> dirs;
        dirs.reserve(static_cast<size_t>(std::min<uint64_t>(N_LON * N_LAT, MAX_POINTS_CAP)));

        // 取格子中心进行采样：
        // - 经度 λ ∈ [0, 2π)，把 0..360° 等分为 N_LON_TARGET 份，抽稀步进为 lon_step
        // - 纬度 φ  ∈ [-π/2, +π/2]，把 0..180° 等分为 N_LAT_TARGET 份，抽稀步进为 lat_step
        //   注意：题目中的“0..180°（纬度）”通常对应极角 θ ∈ [0, π]，与常用纬度 φ 关系是 φ = θ - π/2
        const long double two_pi = 2.0L * M_PI;
        const long double d_lambda = two_pi / static_cast<long double>(N_LON_TARGET);
        const long double d_theta = M_PI / static_cast<long double>(N_LAT_TARGET); // θ: 0..π

        for (uint64_t ilat = 0; ilat < N_LAT_TARGET; ilat += lat_step) {
            // 以格子中心采样：θ = (i + 0.5) * Δθ
            long double theta = (static_cast<long double>(ilat) + 0.5L) * d_theta; // 0..π
            // 转为地理纬度 φ = θ - π/2
            long double phi = theta - M_PI_2; // -π/2..+π/2
            long double sinφ = std::sin(phi);
            long double cosφ = std::cos(phi);

            for (uint64_t ilon = 0; ilon < N_LON_TARGET; ilon += lon_step) {
                long double lambda = (static_cast<long double>(ilon) + 0.5L) * d_lambda; // 0..2π
                long double cosλ = std::cos(lambda);
                long double sinλ = std::sin(lambda);

                // 单位球方向（相机为球心，R=1）
                Eigen::Vector3d v(
                    static_cast<double>(cosφ * cosλ),
                    static_cast<double>(sinφ),
                    static_cast<double>(cosφ * sinλ)
                );
                dirs.emplace_back(v);

                // 防御：若达到上限，直接返回（极端情况下仍可能超出一点点）
                if (dirs.size() >= MAX_POINTS_CAP) return dirs;
            }
        }
        return dirs;
    }

    // 返回“世界坐标”的网格点位置
    std::vector<Eigen::Vector3d> GenerateSphereGridPointsWorld(
        double s,
        const std::vector<GridRegion>& regions = {},
        int   min_longitude = 20,
        bool  include_poles = true
    ) const
    {
        auto dirs = GenerateSphereGridDirections(s, regions, min_longitude, include_poles);
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