// hardware_interface.h
#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H

#include "dynamixel_sdk/dynamixel_sdk.h"

#define NUM_JOINTS 18

extern int DXL_MOTOR_IDS[NUM_JOINTS];
extern int JOINT_MINIMUMS[NUM_JOINTS];
extern int JOINT_MAXIMUMS[NUM_JOINTS];

#define ADDR_PRO_OPERATING_MODE      11                // 1 Byte
#define ADDR_PRO_TORQUE_ENABLE       64                // 1 Byte
#define ADDR_PRO_GOAL_POSITION       116               // 4 Bytes
#define ADDR_PRO_PRESENT_POSITION    132               // 4 Bytes
#define ADDR_PRO_GOAL_CURRENT        102               // 2 Bytes
#define ADDR_PRO_PRESENT_CURRENT     126               // 2 Bytes
#define ADDR_PRO_GOAL_VELOCITY       104               // 4 Bytes
#define ADDR_PRO_PROFILE_VELOCITY    112               // 4 Bytes
#define ADDR_PRO_PRESENT_TEMPERATURE 146               // 1 Byte

// Data Byte Length
#define LEN_PRO_GOAL_POSITION       4
#define LEN_PRO_PRESENT_POSITION    4
#define LEN_PRO_PRESENT_CURRENT     2
#define LEN_PRO_PRESENT_TEMPERATURE 1

// Communication settings:
#define PROTOCOL_VERSION 2.0            // Set the protocol version is used in the Dynamixel
#define BAUDRATE 3000000                // Dynamixel baudrate : 3 MBS

// Set values for global hand behavior. Of course the motors can be set individually 
// as well or receive slow set value changes while configured to fast velocity mode. 
#define DXL_SLOW_VELOCITY   5
#define DXL_MEDIUM_VELOCITY 120
#define DXL_FAST_VELOCITY   445
#define DXL_LIGHT_GRIP      20
#define DXL_MEDIUM_GRIP     100
#define DXL_STRONG_GRIP     500
#define DXL_ULTRA_GRIP      1400

#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

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
        void setupJoints();
        void openComPort();
        void closeConnection();

        int jointLimitCheck(int jointIx, int setPosition);

        void moveJointNo(int jointIx, int setPosition);
        void setJointForces(const std::vector<int> &currents);
        void setJointForce(int jointIx, int setCurrent);
        void setJointSpeeds(const std::vector<int> &speeds);
        void setJointSpeed(int jointIx, int setSpeed);

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

