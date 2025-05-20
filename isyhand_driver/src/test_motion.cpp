#include <rclcpp/rclcpp.hpp>
#include "isyhand_interface/msg/command_joint_pos.hpp"


int main(int argc, char** argv) {
    const double PI = 3.14159265358979323846;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_motion_node");

    auto joint_command_pub = node->create_publisher<isyhand_interface::msg::CommandJointPos>("/isyhand/command_joint_position", 1);
    rclcpp::Rate loop_rate(100);

    std::vector<int> motorIDs = {11, 12, 13, 14, 21, 22, 23, 24, 31, 32, 33, 34, 41, 42, 43, 44, 51, 52};
    std::vector<int> target_positions = {1500,1500,1500,2000,1500,1500,1500,2000,1500,1500,1500,2000,1500,1500,1500,2000,1800,1800};
    std::vector<int> multiple = {200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,100,100};

    isyhand_interface::msg::CommandJointPos msg;
    msg.absolute_positions.resize(target_positions.size());
    // msg.scaled_positions.resize(target_positions.size());
    msg.joint_names.resize(target_positions.size());
    
    for(size_t i=0; i<motorIDs.size(); i++){
        msg.joint_names[i] = "J" + std::to_string(motorIDs[i]);
    }
    
    unsigned incr = 0;
    while(rclcpp::ok()){
        for(size_t i=0;i<motorIDs.size();i++){
            msg.absolute_positions[i] = target_positions[i] + multiple[i]*std::sin(incr*PI/100);
        }
        joint_command_pub->publish(msg);
        incr = (incr + 1) % 200;
        loop_rate.sleep();
    }
    
    return 0;
}
