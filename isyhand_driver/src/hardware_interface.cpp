#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include "isyhand_driver/hardware_interface.h"

int DXL_MOTOR_IDS[18] = {11, 12, 13, 14, 21, 22, 23, 24, 31, 32, 33, 34, 41, 42, 43, 44, 51, 52};
int JOINT_MINIMUMS[18] = {1000,1000,1000,1770,1000,1000,1000,1710,1000,1000,1000,1770,1000,1000,1000,1770,1710,1710};
int JOINT_MAXIMUMS[18] = {2150,2150,2150,2300,2150,2150,2150,2660,2150,2150,2150,2300,2150,2150,2150,2300,2660,2660};


ISyHandDriver::ISyHandDriver (const char *port_name) 
    : portHandler(dynamixel::PortHandler::getPortHandler(port_name)),
      packetHandler(dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION)),
      groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION),
      groupSyncReadPos(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION),
      groupSyncReadCur(portHandler, packetHandler, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT),
      groupSyncReadTmp(portHandler, packetHandler, ADDR_PRO_PRESENT_TEMPERATURE, LEN_PRO_PRESENT_TEMPERATURE),
      jointPosition(NUM_JOINTS),
      jointCurrent(NUM_JOINTS),
      jointTemperature(NUM_JOINTS),
      jointGoalPosition(NUM_JOINTS)
{
    openComPort();

    // Add parameter storage
    for(int i=0; i<NUM_JOINTS; i++){
        if (!groupSyncReadPos.addParam(DXL_MOTOR_IDS[i])){
            std::cout<<"[ID:"<<DXL_MOTOR_IDS[i]<<"] groupSyncReadPos addparam failed\n";
            exit (EXIT_FAILURE);
        }
        if (!groupSyncReadCur.addParam(DXL_MOTOR_IDS[i])){
            std::cout<<"[ID:"<<DXL_MOTOR_IDS[i]<<"] groupSyncReadCur addparam failed\n";
            exit (EXIT_FAILURE);
        }
        if (!groupSyncReadTmp.addParam(DXL_MOTOR_IDS[i])){
            std::cout<<"[ID:"<<DXL_MOTOR_IDS[i]<<"] groupSyncReadTmp addparam failed\n";
            exit (EXIT_FAILURE);
        }
    }

    toggleTorques(0);
    for(int i=0; i<NUM_JOINTS; i++){
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler,DXL_MOTOR_IDS[i],ADDR_PRO_OPERATING_MODE,5, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS){
            std::cout<<packetHandler->getTxRxResult(dxl_comm_result);
        }
    }
    readJointEncoders();
    jointGoalPosition = jointPosition;
    toggleTorques(1);

    std::cout<<"[ISYHAND]: Finished setup.\n";
};

ISyHandDriver::~ISyHandDriver() {
    toggleTorques(0);
    closeConnection();
}

void ISyHandDriver::openComPort () {
    std::cout<<"[ISYHAND]: Opening port... ";
    if (portHandler->openPort()){
        std::cout<<"[ISYHAND]: Succeeded to open the port\n";
    }
    else {
        std::cout<<"[ISYHAND]: Failed to open the port!\n";
        throw std::runtime_error("[ISYHAND]: Failed to open the port");
    }
    if (portHandler->setBaudRate(BAUDRATE)){
        std::cout<<"[ISYHAND]: Succeeded to change the baudrate!\n";
    }
    else {
        std::cout<<"[ISYHAND]: Failed to change the baudrate!\n";
        throw std::runtime_error("[ISYHAND]: Failed to change the baudrate");
    }

};

void ISyHandDriver::closeConnection() {
    // Disable Dynamixel Torque
    for (int i = 0; i < NUM_JOINTS; i++){
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_MOTOR_IDS[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            std::cout<<packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
            std::cout<<packetHandler->getRxPacketError(dxl_error);
        }
    }
    // Close port
    portHandler->closePort();
};

void ISyHandDriver::toggleTorques(bool turn_on){
    for(int i=0; i<NUM_JOINTS; i++){
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler,DXL_MOTOR_IDS[i],ADDR_PRO_TORQUE_ENABLE,turn_on,&dxl_error);
        if (dxl_comm_result != COMM_SUCCESS){
            std::cout<<packetHandler->getTxRxResult(dxl_comm_result);
        }
    }
}

int ISyHandDriver::jointLimitCheck(int jointIx, int setPosition){
    return std::max(JOINT_MINIMUMS[jointIx],std::min(JOINT_MAXIMUMS[jointIx],setPosition));
}

void ISyHandDriver::moveJointNo(int jointIx, int setPosition){
    setPosition = jointLimitCheck(jointIx,setPosition);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler,DXL_MOTOR_IDS[jointIx],ADDR_PRO_GOAL_POSITION,setPosition,&dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
        std::cout<<packetHandler->getTxRxResult(dxl_comm_result)<<"\n";
    }
}

void ISyHandDriver::setJointForces(const std::vector<int>& currents) {
    if (currents.size() != NUM_JOINTS) {
        std::cerr << "[ISYHAND]: Warning: Incorrect number of current values. Expected " << NUM_JOINTS 
                  << ", got " << currents.size() << std::endl;
        return;
    }
    for (int i = 0; i < NUM_JOINTS; i++) {
        if(currents[i]>0){
            setJointForce(i, currents[i]);
        }
    }
}

void ISyHandDriver::setJointForce(int jointIx, int setCurrent){
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler,DXL_MOTOR_IDS[jointIx],ADDR_PRO_GOAL_CURRENT,setCurrent,&dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
        std::cout<<packetHandler->getTxRxResult(dxl_comm_result)<<"\n";
    } 
}

void ISyHandDriver::setJointSpeeds(const std::vector<int>& speeds) {
    if (speeds.size() != NUM_JOINTS) {
        std::cerr << "[ISYHAND]: Warning: Incorrect number of speed values. Expected " << NUM_JOINTS 
                  << ", got " << speeds.size() << std::endl;
        return;
    }
    for (int i = 0; i < NUM_JOINTS; i++) {
        setJointSpeed(i, speeds[i]);
    }
}

void ISyHandDriver::setJointSpeed(int jointIx, int setSpeed){
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler,DXL_MOTOR_IDS[jointIx],ADDR_PRO_PROFILE_VELOCITY,setSpeed,&dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
        std::cout<<packetHandler->getTxRxResult(dxl_comm_result)<<"\n";
    } 
}

void ISyHandDriver::writeJointPositions(const std::vector<int>& goalPositions){
    const std::vector<int>& positions = goalPositions.empty() ? jointGoalPosition : goalPositions;
    uint8_t param_goal_position[4];
    for(int i = 0; i < NUM_JOINTS; i++){
        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(positions[i]));
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(positions[i]));
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(positions[i]));
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(positions[i]));

        if (!groupSyncWrite.addParam(DXL_MOTOR_IDS[i], param_goal_position)){
            std::cout<<"[ISYHAND]: [ID: "<<DXL_MOTOR_IDS[i]<<" groupSyncWrite addparam failed.\n";
            exit (EXIT_FAILURE);
        }
    }

    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS){
        std::cout<<packetHandler->getTxRxResult(dxl_comm_result)<<"\n";
    }

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();
}
void ISyHandDriver::readJointTemperatures(){
    readJointValues(ADDR_PRO_PRESENT_TEMPERATURE, LEN_PRO_PRESENT_TEMPERATURE, &groupSyncReadTmp, jointTemperature);
    // std::cout<<"TEMPERATURES:\n";
    // for (int i = 0; i<NUM_JOINTS;i++){
    //     std::cout<<"Motor "<<DXL_MOTOR_IDS[i]<<": "<<jointTemperature[i]<<"\n";
    // }
}

void ISyHandDriver::readJointCurrents(){
    readJointValues(ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT, &groupSyncReadCur, jointCurrent);
    // std::cout<<"CURRENTS:\n";
    // for (int i = 0; i<NUM_JOINTS;i++){
    //     std::cout<<"Motor "<<DXL_MOTOR_IDS[i]<<": "<<jointCurrent[i]<<"\n";
    // }
}

void ISyHandDriver::readJointEncoders(){
    readJointValues(ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION, &groupSyncReadPos, jointPosition);
    // std::cout<<"POSITIONS:\n";
    // for (int i = 0; i<NUM_JOINTS;i++){
    //     std::cout<<"Motor "<<DXL_MOTOR_IDS[i]<<": "<<jointPosition[i]<<"\n";
    // }
}

void ISyHandDriver::readJointValues(int address, int length, dynamixel::GroupSyncRead* reader, std::vector<int>& storage){
    bool dxl_getdata_result;
    dxl_comm_result = reader->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS){
        std::cout<<"[ISYHAND]: Read failed: "<<packetHandler->getTxRxResult(dxl_comm_result)<<"\n";
    }
    else {
        for (int i = 0; i < NUM_JOINTS; i++){
            dxl_getdata_result = reader->isAvailable(DXL_MOTOR_IDS[i],address,length);
            if (!dxl_getdata_result){
                std::cout<<"[ISYHAND]: [ID:"<<DXL_MOTOR_IDS[i]<<"] groupSyncRead getdata failed\n";
            }
            else{
                storage[i] = reader->getData(DXL_MOTOR_IDS[i],address,length);
            }
        }
    }
}

// int main(int argc, char * argv[]){
//     ISyHandDriver isyhand("/dev/ttyUSB0");
//     isyhand.readJointEncoders();
//     isyhand.readJointTemperatures();
//     isyhand.readJointCurrents();
    
//     return 0;
// }