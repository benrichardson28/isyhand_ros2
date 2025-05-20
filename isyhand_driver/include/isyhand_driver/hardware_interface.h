// hardware_interface.h
#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H

#include "dynamixel_sdk/dynamixel_sdk.h"
#include <unordered_map>
#include <optional>

#define NUM_JOINTS 18

#define ADDR_PRO_OPERATING_MODE      11                // 1 Byte
#define ADDR_PRO_TORQUE_ENABLE       64                // 1 Byte
#define ADDR_PRO_GOAL_POSITION       116               // 4 Bytes
#define ADDR_PRO_PRESENT_POSITION    132               // 4 Bytes
#define ADDR_PRO_GOAL_CURRENT        102               // 2 Bytes
#define ADDR_PRO_PRESENT_CURRENT     126               // 2 Bytes
#define ADDR_PRO_GOAL_VELOCITY       104               // 4 Bytes
#define ADDR_PRO_PROFILE_VELOCITY    112               // 4 Bytes
#define ADDR_PRO_PRESENT_TEMPERATURE 146               // 1 Byte
#define ADDR_POS_D_GAIN              80
#define ADDR_POS_I_GAIN              82
#define ADDR_POS_P_GAIN              84

// Data Byte Length
#define LEN_PRO_GOAL_POSITION       4
#define LEN_PRO_PRESENT_POSITION    4
#define LEN_PRO_PRESENT_CURRENT     2
#define LEN_PRO_PRESENT_TEMPERATURE 1

// Communication settings:
#define PROTOCOL_VERSION 2.0            // Set the protocol version is used in the Dynamixel
#define BAUDRATE 3000000                // Dynamixel baudrate : 3 MBS

class ISyHandDriver {

    const char *port_name;
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    
    dynamixel::GroupSyncWrite groupSyncWrite;
    dynamixel::GroupSyncRead groupSyncReadPos;
    dynamixel::GroupSyncRead groupSyncReadCur;
    dynamixel::GroupSyncRead groupSyncReadTmp;

    std::vector<int> jointPosition;
    std::vector<int> jointCurrent;
    std::vector<int> jointTemperature;

    std::vector<int> jointGoalPosition;

    bool dxl_comm_result;
    uint8_t dxl_error;

    void readJointValues(int address, int length, dynamixel::GroupSyncRead* reader, std::vector<int>& storage);

    void toggleTorques(bool turn_on);
    
    public:
        ISyHandDriver(const char *port_name);
        ~ISyHandDriver();

        std::unordered_map<std::string, int> jointMap;
        std::vector<int> dxlMotorIds;
        std::vector<int> jointMins;
        std::vector<int> jointMaxs;
        std::vector<bool> jointInversion;
        std::vector<int> startingPosition;
        
        void setupJoints();
        void openComPort();
        void closeConnection();

        int jointLimitCheck(int jointIx, int setPosition);

        void moveJointNo(int jointIx, int setPosition);
        void setJointForces(const std::vector<int> &currents);
        void setJointForce(int jointIx, int setCurrent);
        void setJointSpeeds(const std::vector<int> &speeds);
        void setJointSpeed(int jointIx, int setSpeed);

        void setGains(std::optional<uint16_t> d_gain = std::nullopt, 
                      std::optional<uint16_t> i_gain = std::nullopt, 
                      std::optional<uint16_t> p_gain = std::nullopt);

        void readJointTemperatures();
        void readJointCurrents();
        void readJointEncoders();

        void setJointPositions();
        void writeJointPositions(const std::vector<int>& goalPositions = std::vector<int>());

        inline std::vector<int> getJointPosition() {return jointPosition;}
        inline std::vector<int> getJointCurrent() {return jointCurrent;}
        inline std::vector<int> getJointTemperature() {return jointTemperature;}





};


#endif // HARDWARE_INTERFACE_H

