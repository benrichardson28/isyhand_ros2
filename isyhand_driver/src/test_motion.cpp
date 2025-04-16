#include <rclcpp/rclcpp.hpp>
#include "isyhand_driver/msg/command_joint_pos.hpp"


int main(int argc, char** argv) {
    const double PI = 3.14159265358979323846;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_motion_node");

    auto joint_command_pub = node->create_publisher<isyhand_driver::msg::CommandJointPos>("set_joint_values", 1);
    rclcpp::Rate loop_rate(100);

    std::vector<int> target_positions = {1500,1500,1500,2000,1500,1500,1500,2000,1500,1500,1500,2000,1500,1500,1500,2000,1800,1800};
    std::vector<int> multiple = {200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,100,100};

    isyhand_driver::msg::CommandJointPos msg;
    msg.positions.resize(target_positions.size());


    unsigned incr = 0;
    while(rclcpp::ok()){
        for(size_t i=0;i<target_positions.size();i++){
            msg.positions[i] = target_positions[i] + multiple[i]*std::sin(incr*PI/100);
        }
        joint_command_pub->publish(msg);
        incr = (incr + 1) % 200;
        loop_rate.sleep();
    }
    
    return 0;
}
