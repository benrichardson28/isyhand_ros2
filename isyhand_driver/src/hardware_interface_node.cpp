#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <urdf/model.h>
#include "isyhand_interface/msg/joint_state.hpp"
#include "isyhand_interface/msg/command_joint_pos.hpp"
#include <isyhand_driver/hardware_interface.h>
#include <set>
#include <mutex>

class ISyHandROSNode : public rclcpp::Node {
private:
    ISyHandDriver isyhand;  // Hand hardware driver
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_pos_srv_;
    rclcpp::Publisher<isyhand_interface::msg::JointState>::SharedPtr joint_state_pub;
    rclcpp::Subscription<isyhand_interface::msg::CommandJointPos>::SharedPtr joint_command_sub;
    rclcpp::TimerBase::SharedPtr timer_;  // Loop frequency (Hz)

    std::vector<int> target_positions;
    // std::vector<int> set_joint_speeds, set_joint_currents;
    bool publish_position, publish_current, publish_temperature;

    std::unordered_map<std::string, std::pair<double, double>> joint_limits_radians;
    std::unordered_map<std::string, std::pair<double, double>> joint_limits_eff_vel;

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
        
        uint16_t d_gain = static_cast<uint16_t>(this->declare_parameter("pos_d_gain",600));
        uint16_t i_gain = static_cast<uint16_t>(this->declare_parameter("pos_i_gain",0));
        uint16_t p_gain = static_cast<uint16_t>(this->declare_parameter("pos_p_gain",250));
        isyhand.setGains(d_gain,i_gain,p_gain);
        this->declare_parameter<std::string>("urdf_filepath",std::string());

        // this->declare_parameter<std::vector<int>>("set_joint_speeds",std::vector<int>()); //optional would be to set to 300/400 as is currently set
        // this->declare_parameter<std::vector<int>>("set_joint_currents",std::vector<int>()); //optional would be to set to 1750 (genuine max) as is currently set

        // std::vector<long> tmp_joint_speeds = this->get_parameter("set_joint_speeds").as_integer_array();
        // set_joint_speeds = std::vector<int>(begin(tmp_joint_speeds),end(tmp_joint_speeds));

        // std::vector<long> tmp_joint_currents = this->get_parameter("set_joint_currents").as_integer_array();
        // set_joint_currents = std::vector<int>(begin(tmp_joint_currents),end(tmp_joint_currents));

        // if( ! set_joint_speeds.empty() ) {
        //     if (set_joint_speeds.size() == 1){
        //         set_joint_speeds.assign(NUM_JOINTS, set_joint_speeds[0]);
        //         isyhand.setJointSpeeds(set_joint_speeds);
        //     } else if (set_joint_speeds.size() != NUM_JOINTS) {
        //         RCLCPP_WARN(this->get_logger(),"set_joint_speeds parameter has incorrect length. Ignoring.");
        //         set_joint_speeds.clear();
        //     } else {
        //         isyhand.setJointSpeeds(set_joint_speeds);
        //     }
        // }

        // if( ! set_joint_currents.empty() ) {
        //     if (set_joint_currents.size() == 1) {
        //         set_joint_currents.assign(NUM_JOINTS, set_joint_currents[0]); 
        //         isyhand.setJointForces(set_joint_currents);
        //     } else if (set_joint_currents.size() != NUM_JOINTS) {
        //         RCLCPP_WARN(this->get_logger(),"set_joint_currents parameter has incorrect length. Ignoring.");
        //         set_joint_currents.clear();
        //     } else {
        //         isyhand.setJointForces(set_joint_currents);
        //     }
        // }

        // important to remember, radians and encoder values are inverse
        // that is, min radians = max encoder; max radians = min encoder
        extractJointInfo();

        setIsyHandJointProps();

        // Services:
        reset_pos_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "isyhand/reset_position",
            std::bind(&ISyHandROSNode::resetPositionCallback, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Publisher: Joint state feedback
        joint_state_pub = this->create_publisher<isyhand_interface::msg::JointState>("isyhand/joint_states", 1);

        // Subscriber: Joint command input
        joint_command_sub = this->create_subscription<isyhand_interface::msg::CommandJointPos>(
            "isyhand/command_joint_position", 1, 
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

        for(int i=0;i<NUM_JOINTS;i++){
            std::cout<<isyhand.dxlMotorIds[i]<<"(min, max): "<<isyhand.jointMins[i]<<", "<<isyhand.jointMaxs[i]<<"\n";
        }
    }

    void extractJointInfo() {
        std::string urdf_string;
        if (!this->get_parameter("urdf_filepath", urdf_string)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get 'urdf_filepath' parameter");
        } else {
            urdf::Model model;
            if (!model.initFile(urdf_string)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF string into urdf::Model");
            } else {
                RCLCPP_INFO(this->get_logger(), "URDF successfully loaded with %ld joints", model.joints_.size());

                for (const auto& [name, joint] : model.joints_) {
                    if ((joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::PRISMATIC) && joint->limits) {
                        joint_limits_radians[name] = {joint->limits->lower, joint->limits->upper};
                        joint_limits_eff_vel[name] = {joint->limits->effort,joint->limits->velocity};
                    }
                }
            }
        }
    }

    void setIsyHandJointProps(){
        float tmp_eff = 0.0;
        float tmp_vel = 0.0;
        float effort_conversion = 0.0;
        float velocity_conversion = 60*300/(68.66*2*3.14159);
        std::vector<int> joint_currents (NUM_JOINTS, 0);
        std::vector<int> joint_speeds (NUM_JOINTS, 0);
        std::set<std::string> dyna_xls = {"J11","J21","J31","J41","J12","J22","J32","J42","J13","J23","J33","J43"};
        std::set<std::string> dyna_xcs = {"J14","J24","J34","J44","J51","J52"};
        for(int i=0; i<NUM_JOINTS; i++){
            std::string joint_name = "J" + std::to_string(isyhand.dxlMotorIds[i]);
            tmp_eff = joint_limits_eff_vel[joint_name].first;
            tmp_vel = joint_limits_eff_vel[joint_name].second;
            if (dyna_xls.count(joint_name)){
                // effort_conversion = 368 / 0.52;
                effort_conversion = 1470 / 0.52;
            } else if (dyna_xcs.count(joint_name)){
                // effort_conversion = 450 / 0.93;
                effort_conversion = 1800 / 0.93;
            } 
            joint_currents[i] = static_cast<int>(effort_conversion*tmp_eff);
            joint_speeds[i] = static_cast<int>(velocity_conversion*tmp_vel);
            
            if(isyhand.jointInversion[i]){
                isyhand.jointMins[i] = isyhand.startingPosition[i] - static_cast<int>(joint_limits_radians[joint_name].second * 180 / 3.141592653 / .087891);
                isyhand.jointMaxs[i] = isyhand.startingPosition[i] - static_cast<int>(joint_limits_radians[joint_name].first * 180 / 3.141592653 / .087891);;
            } else{
                isyhand.jointMins[i] = isyhand.startingPosition[i] + static_cast<int>(joint_limits_radians[joint_name].first * 180 / 3.141592653 / .087891);
                isyhand.jointMaxs[i] = isyhand.startingPosition[i] + static_cast<int>(joint_limits_radians[joint_name].second * 180 / 3.141592653 / .087891);;
            }
        }
        
        isyhand.setJointForces(joint_currents);
        isyhand.setJointSpeeds(joint_speeds);
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
        msg.header.stamp = this->get_clock()->now();
        msg.joint_names.resize(isyhand.dxlMotorIds.size());
        for(int i=0; i<NUM_JOINTS; i++){
            msg.joint_names[i] = "J" + std::to_string(isyhand.dxlMotorIds[i]);
        }

        if (publish_position) {
            isyhand.readJointEncoders();
            msg.absolute_positions = isyhand.getJointPosition();
            msg.scale_positions = encoderToScale(msg.absolute_positions);
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
        std::vector<int> temp_positions(NUM_JOINTS);
        if ((msg->joint_names.size() != NUM_JOINTS) || 
            ((msg->absolute_positions.size() != NUM_JOINTS) && 
            (msg->scaled_positions.size() != NUM_JOINTS))) {
            RCLCPP_ERROR(this->get_logger(),"Received joint command with incorrect number of joints!");
            return;
        }

        if (msg->absolute_positions.size() == NUM_JOINTS){
            const auto& msg_positions = msg->absolute_positions;
            for(int i=0; i<NUM_JOINTS; i++){
                temp_positions[isyhand.jointMap[msg->joint_names[i]]] = msg_positions[i];
            }
        }
        else {
            const auto& msg_positions = msg->scaled_positions;
            std::vector<float> rescaled_positions(NUM_JOINTS);
            for(int i=0; i<NUM_JOINTS; i++){
                rescaled_positions[isyhand.jointMap[msg->joint_names[i]]] = msg_positions[i];
            }
            temp_positions = scaleToEncoder(rescaled_positions);
        }
        
        std::lock_guard<std::mutex> lock(target_positions_mutex);
        target_positions = temp_positions;
    }

    /**
     * The incoming scale should be low in rad position to high in rad position,
     * which is the opposite of the encoder positions. So need to convert
     * to 1-scale_value
     */
    std::vector<int> scaleToEncoder(std::vector<float> scaled_positions){
        std::vector<int> encoder_positions(scaled_positions.size(), 0);
        for(int i=0; i<NUM_JOINTS; i++){
            float scale_val = std::clamp(scaled_positions[i],-1.0f,1.0f);
            scale_val = isyhand.jointInversion[i] ? 1.0f - (scale_val + 1.0f) / 2.0f : (scale_val + 1.0f) / 2.0f ;
            encoder_positions[i] = scale_val * (isyhand.jointMaxs[i]-isyhand.jointMins[i]) + isyhand.jointMins[i];
        }
        return encoder_positions;
    }

    std::vector<float> encoderToScale(std::vector<int> encoder_positions) {
        std::vector<float> scaled_positions(encoder_positions.size(), 0.0f);
        for(int i = 0; i < NUM_JOINTS; i++) {
            float enc_val = static_cast<float>(encoder_positions[i]);
            // Normalize encoder value between 0 and 1
            float norm_val = (enc_val - isyhand.jointMins[i]) / (isyhand.jointMaxs[i] - isyhand.jointMins[i]);
            // Undo inversion if necessary
            float scale_val = isyhand.jointInversion[i] ? 1.0f - 2.0f * norm_val : 2.0f * norm_val - 1.0f;
            // Clamp just in case
            scaled_positions[i] = std::clamp(scale_val, -1.0f, 1.0f);
        }
        return scaled_positions;
    }
    /**
     * To go to encoder value to urdf radians, we have to invert the relationship.
     * So we can see where in the encoder range we are from 0 to 1, then invert 
     * that to convert to radians.  NOT SURE THIS IS NEEDED AT ALL
     */
    std::vector<float> encoderToRadians(std::vector<int> encoder_positions){
        std::vector<float> radian_positions(encoder_positions.size());
        for(int i=0; i<NUM_JOINTS; i++){
            // as we approach max encoder, this goes to 1.
            float scale_val = static_cast<float>(encoder_positions[i] - isyhand.jointMins[i]) / (isyhand.jointMaxs[i] - isyhand.jointMins[i]);
            scale_val = isyhand.jointInversion[i] ? (1.0f - scale_val) : scale_val;
            std::pair<double, double> jointRadLim = joint_limits_radians["J" + std::to_string(isyhand.dxlMotorIds[i])];
            radian_positions[i] = scale_val * (jointRadLim.second - jointRadLim.first) + jointRadLim.first;
        }
        return radian_positions;
    }

    void resetPositionCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> ,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(target_positions_mutex);
        target_positions = isyhand.startingPosition;
        response->success = true;
        response->message = "Joint positions reset to starting position.";
        RCLCPP_INFO(this->get_logger(), "Received reset request. Positions reset.");
    }
    
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ISyHandROSNode>());
    rclcpp::shutdown();
    return 0;
}
