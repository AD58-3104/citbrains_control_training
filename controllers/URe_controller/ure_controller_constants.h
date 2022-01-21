#ifndef URE_CONSTANTS_H
#define URE_CONSTANTS_H

/* ur10e's dimention

https://www.universal-robots.com/media/1810584/05_2021_10e_factsheet_jp.pdf

plane to base x = + 0mm, z = 0mm
base to first link x = + 176mm, z = + 181mm
first to second link x = + 0, z = + 613mm
second to third link x = - 137mm, z = + 571mm
thrid to wrist 1 x = + 135mm, z = + 0mm
wrist 1 to wrist 2 x = + 0mm,z = + 120mm
wrist 2 to wrist 3 x = + 117mm, z = + 0mm
*/


namespace UreController_constants
{

    enum class FingerNum
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
        finger_middle_joint_3,
        length
    };
    enum class ArmNum
    {
        shoulder_pan_joint = 0,
        shoulder_lift_joint,
        elbow_joint,
        wrist_1_joint,
        wrist_2_joint,
        wrist_3_joint,
        length
    };
    enum class SensorNum
    {
        elbow_joint_sensor = 0,
        shoulder_lift_joint_sensor,
        shoulder_pan_joint_sensor,
        wrist_1_joint_sensor,
        wrist_2_joint_sensor,
        wrist_3_joint_sensor,
        length
    };
    const std::vector<std::string> joint_name_list{
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"};
    const std::vector<std::string> sensor_name_list{
        "shoulder_pan_joint_sensor",
        "shoulder_lift_joint_sensor",
        "elbow_joint_sensor",
        "wrist_1_joint_sensor",
        "wrist_2_joint_sensor",
        "wrist_3_joint_sensor"};
    const std::vector<std::string> finger_name_list{
        "finger_1_joint_1",
        "finger_2_joint_1",
        "finger_middle_joint_1"};
    inline static constexpr int32_t DEGREE_RECEIVE_PORT = 7650;
    inline static constexpr int32_t SENSOR_SEND_PORT = 7651;
}
#endif