/*
@brief: 弹道解算 适配陈君的rm_vision
@author: CodeAlan  华南师大Vanguard战队
*/
// 近点只考虑水平方向的空气阻力

//TODO 完整弹道模型
//TODO 适配英雄机器人弹道解算

// STD
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>
#include <iostream>

#include "SolveTrajectory.hpp"

namespace rm_auto_aim
{
SolveTrajectory::SolveTrajectory(const float &k, const int &bias_time, const float &s_bias, const float &z_bias)
: k(k), bias_time(bias_time), s_bias(s_bias), z_bias(z_bias)
{}

void SolveTrajectory::init(const std::shared_ptr<msg::Velocity> velocity_msg)
{
  current_v = velocity_msg->velocity;
}

/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rachouxiang
@return z:m
*/
float SolveTrajectory::monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z, t;
    //t为给定v与angle时的飞行时间
    t = (float)((exp(k * s) - 1) / (k * v * cos(angle)));
    //z为给定v与angle时的高度
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    return z;
}

/*
@brief 完整弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
//TODO 完整弹道模型
// float SolveTrajectory::completeAirResistanceModel(float s, float v, float angle)
// {
//     // TODO: Implement complete air resistance model
//     return 0.0f;
// }

/*
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/
float SolveTrajectory::pitchTrajectoryCompensation(float s, float z, float v) {
    float z_temp = z;
    float angle_pitch = 0.0f;
    std::cout << "current_velocity_:" << v << std::endl;
    for (int i = 0; i < 20; i++) {
        angle_pitch = std::atan2(z_temp, s);
        float z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
        float dz = 0.3f * (z - z_actual);
        z_temp += dz;

        if (std::fabs(dz) < 0.00001f) {
            break;
        }
    }

    return angle_pitch;
}

/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/
void SolveTrajectory::autoSolveTrajectory(float& pitch, float& yaw, float& aim_x, float& aim_y, float& aim_z, const std::shared_ptr<msg::Target> msg)
{

    tar_yaw = msg->yaw;
    // 线性预测
    float timeDelay = bias_time/1000.0 + fly_time;
    tar_yaw += msg->v_yaw * timeDelay;

    //计算四块装甲板的位置
    //装甲板id顺序，以四块装甲板为例，逆时针编号
    //      2
    //   3     1
    //      0
	int use_1 = 1;
	int i = 0;
    int idx = 0; // 选择的装甲板
    if (msg->armors_num  == ARMOR_NUM_BALANCE) {
        for (i = 0; i<2; i++) {
            float tmp_yaw = tar_yaw + i * PI;
            float r = msg->radius_1;
            tar_position[i].x = msg->position.x - r*cos(tmp_yaw);
            tar_position[i].y = msg->position.y - r*sin(tmp_yaw);
            tar_position[i].z = msg->position.z;
            tar_position[i].yaw = tmp_yaw;
        }

        float yaw_diff_min = fabsf(msg->yaw - tar_position[0].yaw);

        //因为是平衡步兵 只需判断两块装甲板即可
        float temp_yaw_diff = fabsf(msg->yaw- tar_position[1].yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = 1;
        }


    } else if (msg->armors_num  == ARMOR_NUM_OUTPOST) {  //前哨站
        for (i = 0; i<3; i++) {
            float tmp_yaw = tar_yaw + i * 2.0 * PI/3.0;  // 2/3PI
            float r =  (msg->radius_1 + msg->radius_2)/2;   //理论上r1=r2 这里取个平均值
            tar_position[i].x = msg->position.x - r*cos(tmp_yaw);
            tar_position[i].y = msg->position.y - r*sin(tmp_yaw);
            tar_position[i].z = msg->position.z;
            tar_position[i].yaw = tmp_yaw;
        }

        //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用


    } else {
        for (i = 0; i<4; i++) {
            float tmp_yaw = tar_yaw + i * PI/2.0;
            //std::cout << "tmp_yaw: " << tmp_yaw  << std::endl;
            float r = use_1 ? msg->radius_1 : msg->radius_2;
            //std::cout << "r: " << r  << std::endl;
            tar_position[i].x = msg->position.x - r*cos(tmp_yaw);
            //std::cout << "tar_x: " << tar_position[i].x  << std::endl;
            tar_position[i].y = msg->position.y - r*sin(tmp_yaw);
            //std::cout << "tar_y: " << tar_position[i].y  << std::endl;
            tar_position[i].z = use_1 ? msg->position.z : msg->position.z + msg->dz;
            //std::cout << "tar_z: " << tar_position[i].z  << std::endl;
            tar_position[i].yaw = tmp_yaw;
            //std::cout << "tar_yaw: " << tar_position[i].yaw  << std::endl;
            use_1 = !use_1;
            //std::cout << "use_1: " << use_1  << std::endl;
        }
     

            //2种常见决策方案：
            //1.计算枪管到目标装甲板yaw最小的那个装甲板
            //2.计算距离最近的装甲板

            //计算距离最近的装甲板
        //	float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
        //	int idx = 0;
        //	for (i = 1; i<4; i++)
        //	{
        //		float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
        //		if (temp_dis_diff < dis_diff_min)
        //		{
        //			dis_diff_min = temp_dis_diff;
        //			idx = i;
        //		}
        //	}
        //

            //计算枪管到目标装甲板yaw最小的那个装甲板
        float yaw_diff_min = fabsf(msg->yaw - tar_position[0].yaw);
        for (i = 0; i<4; i++) {
            float temp_yaw_diff = fabsf(msg->yaw - tar_position[i].yaw);
            if (temp_yaw_diff < yaw_diff_min)
            {
                yaw_diff_min = temp_yaw_diff;
                idx = i;
            }
        }

    }

	

    aim_z = tar_position[idx].z + msg->velocity.z * timeDelay;
    std::cout << "aim_z: " << aim_z  << std::endl;
    aim_x = tar_position[idx].x + msg->velocity.x * timeDelay;
    std::cout << "aim_x: " << aim_x  << std::endl;
    aim_y = tar_position[idx].y + msg->velocity.y * timeDelay;
    std::cout << "aim_y: " << aim_y  << std::endl;

    //这里符号给错了
    pitch = -pitchTrajectoryCompensation(sqrt((aim_x) * (aim_x) + (aim_y) * (aim_y)) - s_bias,
            aim_z + z_bias, current_v);

    std::cout << "pitch: " << pitch  << std::endl;
    yaw = (float)(atan2(aim_y, aim_x));

}

// 从坐标轴正向看向原点，逆时针方向为正

}
