#include "sUDPSocket.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include "ure.pb.h"

using namespace std::literals::chrono_literals;
enum FingerNum
{
    palm_finger_1_joint = 0,
    finger_1_joint_1,
    finger_1_joint_2,
    finger_1_joint_3,
    palm_finger_2_joint,
    finger_2_joint_1,
    finger_2_joint_2,
    finger_2_joint_3,
    finger_middle_joint_1,
    finger_middle_joint_2,
    finger_middle_joint_3
};
enum ArmNum
{
    shoulder_pan_joint = 0,
    shoulder_lift_joint,
    elbow_joint,
    wrist_1_joint,
    wrist_2_joint,
    wrist_3_joint
};
enum SensorNum
{
    elbow_joint_sensor = 0,
    shoulder_lift_joint_sensor,
    shoulder_pan_joint_sensor,
    wrist_1_joint_sensor,
    wrist_2_joint_sensor,
    wrist_3_joint_sensor
};

const std::vector<std::string> finger_name_list{
    "finger_1_joint_1",
    "finger_2_joint_1",
    "finger_middle_joint_1"};
const std::vector<std::string> joint_name_list{
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"};

int main(int argc, char const *argv[])
{
    Citbrains::Udpsocket::UDPClient client("127.0.0.1", 7650, Citbrains::Udpsocket::SocketMode::unicast_mode);
    auto to_rad = [](float deg)
    {
        return deg / 180 * 3.1415926;
    };
    std::cout << "start" << std::endl;
    for (int_fast64_t loop = 0; loop < 100; loop++)
    {
        UreMessage::JointDegree deg;
        for (size_t i = 0; i < joint_name_list.size(); ++i)
        {
            deg.add_arm_degree_list(to_rad(-(loop % 30)));
        }
        for (size_t i = 0; i < finger_name_list.size(); ++i)
        {
            deg.add_finger_degree_list(to_rad(0));
        }
        client.send(std::move(deg.SerializeAsString()));
        std::this_thread::sleep_for(32ms);
    }
    int cnt = 0;
    int plus = -1;
    for (int_fast64_t j = 0;; ++j)
    {
        UreMessage::JointDegree deg;
        if ((cnt == 0) || (cnt == 360))
        {
            plus *= -1;
        }
        cnt += plus;
        for (size_t i = 0; i < joint_name_list.size(); ++i)
        {
            if (i == 0)
            {
                deg.add_arm_degree_list(to_rad(cnt));
            }
            else
            {
                deg.add_arm_degree_list(to_rad(-30));
            }
        }
        for (size_t i = 0; i < finger_name_list.size(); ++i)
        {
            deg.add_finger_degree_list(to_rad(0));
        }
        client.send(std::move(deg.SerializeAsString()));
        std::this_thread::sleep_for(32ms);
    }

    return 0;
}
