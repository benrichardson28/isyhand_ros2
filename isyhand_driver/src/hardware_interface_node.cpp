#include <rclcpp/rclcpp.hpp>
#include "isyhand_interface/msg/joint_state.hpp"
#include "isyhand_interface/msg/command_joint_pos.hpp"
#include <isyhand_driver/hardware_interface.h>
#include <mutex>

class ISyHandROSNode : public rclcpp::Node {
private:
    ISyHandDriver isyhand;  // Hand hardware driver
    rclcpp::Publisher<isyhand_interface::msg::JointState>::SharedPtr joint_state_pub;
    rclcpp::Subscription<isyhand_interface::msg::CommandJointPos>::SharedPtr joint_command_sub;
    rclcpp::TimerBase::SharedPtr timer_;  // Loop frequency (Hz)

    std::vector<int> target_positions;
    std::vector<int> set_joint_speeds, set_joint_currents;
    bool publish_position, publish_current, publish_temperature;

    bool write_cycle = true;
    std::mutex target_positions_mutex;  // Mutex for thread safety

public:
    ISyHandROSNode() 
    : Node("isyhand_driver_node"),
      isyhand("/dev/ttyUSB0")
    {
        // Get ROS parameters for data selection
        publish_position = this->declare_parameter("publish_position", true);
        publish_current = this->declare_parameter("publish_current", false);
        publish_temperature = this->declare_parameter("publish_temperature", false);

        this->declare_parameter<std::vector<int>>("set_joint_speeds",std::vector<int>()); //optional would be to set to 300/400 as is currently set
        std::vector<long> tmp_joint_speeds = this->get_parameter("set_joint_speeds").as_integer_array();
        set_joint_speeds = std::vector<int>(begin(tmp_joint_speeds),end(tmp_joint_speeds));

        this->declare_parameter<std::vector<int>>("set_joint_currents",std::vector<int>()); //optional would be to set to 1750 (genuine max) as is currently set
        std::vector<long> tmp_joint_currents = this->get_parameter("set_joint_currents").as_integer_array();
        set_joint_currents = std::vector<int>(begin(tmp_joint_currents),end(tmp_joint_currents));

        if( ! set_joint_speeds.empty() ) {
            if (set_joint_speeds.size() == 1){
                set_joint_speeds.assign(NUM_JOINTS, set_joint_speeds[0]);
                isyhand.setJointSpeeds(set_joint_speeds);
            } else if (set_joint_speeds.size() != NUM_JOINTS) {
                RCLCPP_WARN(this->get_logger(),"set_joint_speeds parameter has incorrect length. Ignoring.");
                set_joint_speeds.clear();
            } else {
                isyhand.setJointSpeeds(set_joint_speeds);
            }
        }

        if( ! set_joint_currents.empty() ) {
            if (set_joint_currents.size() == 1) {
                set_joint_currents.assign(NUM_JOINTS, set_joint_currents[0]); 
                isyhand.setJointForces(set_joint_currents);
            } else if (set_joint_currents.size() != NUM_JOINTS) {
                RCLCPP_WARN(this->get_logger(),"set_joint_currents parameter has incorrect length. Ignoring.");
                set_joint_currents.clear();
            } else {
                isyhand.setJointForces(set_joint_currents);
            }
        }


        // Publisher: Joint state feedback
        joint_state_pub = this->create_publisher<isyhand_interface::msg::JointState>("joint_states", 10);

        // Subscriber: Joint command input
        joint_command_sub = this->create_subscription<isyhand_interface::msg::CommandJointPos>(
            "set_joint_values", 10, 
            std::bind(&ISyHandROSNode::setJointValues, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),  // 200Hz
            std::bind(&ISyHandROSNode::updateCycle, this)
        );

        RCLCPP_INFO(this->get_logger(),
            "ISyHandROSNode initialized! Publishing Position: %d, Current: %d, Temperature: %d",
            publish_position, publish_current, publish_temperature
        );
    }

    void updateCycle() {
        if (write_cycle) {
            std::lock_guard<std::mutex> lock(target_positions_mutex);  // Lock before accessing target_positions
            isyhand.writeJointPositions(target_positions);  
        } else {
            publishJointStates();  
        }

        write_cycle = !write_cycle;       
    }

    void publishJointStates() {
        isyhand_interface::msg::JointState msg;

        if (publish_position) {
            isyhand.readJointEncoders();
            msg.positions = isyhand.getJointPosition();
        }
        if (publish_current) {
            isyhand.readJointCurrents();
            msg.currents = isyhand.getJointCurrent();
        }
        if (publish_temperature) {
            isyhand.readJointTemperatures();
            msg.temperatures = isyhand.getJointTemperature();
        }

        joint_state_pub->publish(msg);
    }

    void setJointValues(const isyhand_interface::msg::CommandJointPos::SharedPtr msg) {
        if (msg->positions.size() != NUM_JOINTS) {
            RCLCPP_WARN(this->get_logger(),"Received joint command with incorrect number of joints!");
            return;
        }

        std::lock_guard<std::mutex> lock(target_positions_mutex);  // Lock before modifying target_positions
        target_positions = msg->positions;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ISyHandROSNode>());
    rclcpp::shutdown();
    return 0;
}
