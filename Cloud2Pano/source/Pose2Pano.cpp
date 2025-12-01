#define _USE_MATH_DEFINES
#include<fstream> 
#include<vector>
#include<string>
#include "Camera.h"
#include "Pano.h"
#include <cmath>

//读取txt文档 获取相机位姿
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

Eigen::Vector2d Panoprojection(Eigen::Vector3d v,const Camera& camera)
{
    //世界系转相机系

    Eigen::Matrix4d W2C = camera.pose.inverse();
	Eigen::Vector4d v_t = Eigen::Vector4d(v(0), v(1), v(2), 1.0);
    Eigen::Vector4d world2camera = W2C*v;

    //相机系转全景坐标系
    int row = 5844;
	int col = 2922;
    double dis = sqrt(pow(world2camera(0), 2) + pow(world2camera(1), 2) + pow(world2camera(2), 2));  // 计算点到圆心距离
    double theta = atan2(world2camera(1), world2camera(0));  // 方位角 (0,pi)
    double phi = asin(world2camera(2) / dis);  // 俯仰角 [-pi/2,pi/2]
    int x = int((row / 2) * (theta / M_PI));
    int y = int(phi / M_PI) * col;
    int u = row / 2 + x;
    int v = col / 2 - y;
    return Eigen::Vector2d(u, v);
}

int main()
{
    std::string filename = "D:\\experience\\try\\DasModel\\POS\\camera1_pose.asc";
    std::vector<std::vector<double>> translations;
    std::vector<std::vector<double>> quaternions;
    if (readData(filename, translations, quaternions)) 
    {
        const auto CameraPos = Eigen::Vector3d(-6.265600,0.489403,1.539521);
		const auto CameraQuat = Eigen::Quaterniond(-0.502943,- 0.521237,- 0.458408,0.514997);
        Camera camera(CameraPos, CameraQuat);
        for (int i = 0;i< translations.size();i++)
        {
			std::vector<Eigen::Vector2d> new_positions;
            Eigen::Vector3d position(translations[i][0], translations[i][1], translations[i][2]);
            auto new_position=Panoprojection(position, camera);
			new_positions.push_back(new_position);

        }
        
}