#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include "isyhand_driver/hardware_interface.h"

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
      jointGoalPosition(NUM_JOINTS),
      dxlMotorIds{11, 12, 13, 14, 21, 22, 23, 24, 31, 32, 33, 34, 41, 42, 43, 44, 51, 52},
      jointMins{1000,1000,1000,1770,1000,1000,1000,1180,1000,1000,1000,1770,1000,1000,1000,1770,1761,1761},
      jointMaxs{2150,2150,2150,2300,2150,2150,2150,2800,2150,2150,2150,2300,2150,2150,2150,2300,2335,2335},
      jointInversion{1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,0,0},
      startingPosition(18,2048)
{
    for(size_t i=0; i<dxlMotorIds.size(); i++){
        jointMap["J" + std::to_string(dxlMotorIds[i])] = i;
    }
    openComPort();

    // Add parameter storage
    for(int i=0; i<NUM_JOINTS; i++){
        if (!groupSyncReadPos.addParam(dxlMotorIds[i])){
            std::cout<<"[ID:"<<dxlMotorIds[i]<<"] groupSyncReadPos addparam failed\n";
            exit (EXIT_FAILURE);
        }
        if (!groupSyncReadCur.addParam(dxlMotorIds[i])){
            std::cout<<"[ID:"<<dxlMotorIds[i]<<"] groupSyncReadCur addparam failed\n";
            exit (EXIT_FAILURE);
        }
        if (!groupSyncReadTmp.addParam(dxlMotorIds[i])){
            std::cout<<"[ID:"<<dxlMotorIds[i]<<"] groupSyncReadTmp addparam failed\n";
            exit (EXIT_FAILURE);
        }
    }

    toggleTorques(0);
    for(int i=0; i<NUM_JOINTS; i++){
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler,dxlMotorIds[i],ADDR_PRO_OPERATING_MODE,5, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS){
            std::cout<<packetHandler->getTxRxResult(dxl_comm_result);
        }
    }
    jointGoalPosition = startingPosition;
    toggleTorques(1);

    std::cout<<"[ISYHAND]: Finished setup.\n";
};

ISyHandDriver::~ISyHandDriver() {
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
    // toggleTorques(0);
    writeJointPositions(startingPosition);
    // Close port
    portHandler->closePort();
};

void ISyHandDriver::toggleTorques(bool turn_on){
    for(int i=0; i<NUM_JOINTS; i++){
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler,dxlMotorIds[i],ADDR_PRO_TORQUE_ENABLE,turn_on,&dxl_error);
        if (dxl_comm_result != COMM_SUCCESS){
            std::cout<<packetHandler->getTxRxResult(dxl_comm_result);
        }
    }
}

int ISyHandDriver::jointLimitCheck(int jointIx, int setPosition){
    return std::max(jointMins[jointIx],std::min(jointMaxs[jointIx],setPosition));
}

void ISyHandDriver::moveJointNo(int jointIx, int setPosition){
    setPosition = jointLimitCheck(jointIx,setPosition);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler,dxlMotorIds[jointIx],ADDR_PRO_GOAL_POSITION,setPosition,&dxl_error);
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
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler,dxlMotorIds[jointIx],ADDR_PRO_GOAL_CURRENT,setCurrent,&dxl_error);
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
    // std::cout<<"[ISYHAND]: Setting profile velocity for hand\n";
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler,dxlMotorIds[jointIx],ADDR_PRO_PROFILE_VELOCITY,setSpeed,&dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
        std::cout<<packetHandler->getTxRxResult(dxl_comm_result)<<"\n";
    } 
}

void ISyHandDriver::setGains(
    std::optional<uint16_t> d_gain, 
    std::optional<uint16_t> i_gain, 
    std::optional<uint16_t> p_gain
    ) {
        for(int i = 0; i < NUM_JOINTS; i++) {
            if(d_gain.has_value()){
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler,dxlMotorIds[i],ADDR_POS_D_GAIN,d_gain.value(),&dxl_error);
                if (dxl_comm_result != COMM_SUCCESS){
                    std::cout<<packetHandler->getTxRxResult(dxl_comm_result)<<"\n";
                } 
            }
            if(i_gain.has_value()){
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler,dxlMotorIds[i],ADDR_POS_I_GAIN,i_gain.value(),&dxl_error);
                if (dxl_comm_result != COMM_SUCCESS){
                    std::cout<<packetHandler->getTxRxResult(dxl_comm_result)<<"\n";
                } 
            }
            if(p_gain.has_value()){
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler,dxlMotorIds[i],ADDR_POS_P_GAIN,p_gain.value(),&dxl_error);
                if (dxl_comm_result != COMM_SUCCESS){
                    std::cout<<packetHandler->getTxRxResult(dxl_comm_result)<<"\n";
                } 
            }
        }
    }

void ISyHandDriver::writeJointPositions(const std::vector<int>& goalPositions){
    const std::vector<int>& positions = goalPositions.empty() ? jointGoalPosition : goalPositions;
    uint8_t param_goal_position[4];
    std::vector<int> clamped_positions(positions.size());
    for(int i = 0; i < NUM_JOINTS; i++){
        clamped_positions[i] = jointLimitCheck(i,positions[i]);
        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(clamped_positions[i]));
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(clamped_positions[i]));
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(clamped_positions[i]));
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(clamped_positions[i]));

        if (!groupSyncWrite.addParam(dxlMotorIds[i], param_goal_position)){
            std::cout<<"[ISYHAND]: [ID: "<<dxlMotorIds[i]<<" groupSyncWrite addparam failed.\n";
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
    //     std::cout<<"Motor "<<dxlMotorIds[i]<<": "<<jointTemperature[i]<<"\n";
    // }
}

void ISyHandDriver::readJointCurrents(){
    readJointValues(ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT, &groupSyncReadCur, jointCurrent);
    // std::cout<<"CURRENTS:\n";
    // for (int i = 0; i<NUM_JOINTS;i++){
    //     std::cout<<"Motor "<<dxlMotorIds[i]<<": "<<jointCurrent[i]<<"\n";
    // }
}

void ISyHandDriver::readJointEncoders(){
    readJointValues(ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION, &groupSyncReadPos, jointPosition);
    // std::cout<<"POSITIONS:\n";
    // for (int i = 0; i<NUM_JOINTS;i++){
    //     std::cout<<"Motor "<<dxlMotorIds[i]<<": "<<jointPosition[i]<<"\n";
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
            dxl_getdata_result = reader->isAvailable(dxlMotorIds[i],address,length);
            if (!dxl_getdata_result){
                std::cout<<"[ISYHAND]: [ID:"<<dxlMotorIds[i]<<"] groupSyncRead getdata failed\n";
            }
            else{
                storage[i] = reader->getData(dxlMotorIds[i],address,length);
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