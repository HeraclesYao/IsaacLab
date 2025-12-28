#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <cstdio>
#include <memory>
#include <iostream>
#include <vector>
#include <string>
#include <atomic>
#include <chrono>
#include <future>

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
//#include <nlohmann/json.hpp>
#include <stdexcept>

#include <dynamixel_sdk.h>

using namespace dynamixel;

#define DXL_ID_1 0
#define DXL_ID_2 1
#define DXL_ID_3 2
#define DXL_ID_4 3
#define DXL_ID_5 4
#define DXL_ID_6 5
#define DXL_ID_7 6

#define DXL_ID_8 7
#define DXL_ID_9 8
#define DXL_ID_10 9
#define DXL_ID_11 10
#define DXL_ID_12 11
#define DXL_ID_13 12
#define DXL_ID_14 13

#define DXL_POSITION_TO_RADIANS (2.0 * M_PI / 4096.0)

// extern std::vector<double> linkerta_offset;
// extern std::vector<int64_t> linkerta_negation;
// extern int calibration;

// std::atomic<bool> running(true);

class MasterArm
{
public:
    MasterArm(uint8_t num_motors, uint8_t *motor_ids, const std::string &port_name, int baudrate);

    ~MasterArm();

    std::vector<double> getJointAngles();

    std::vector<double> getSyncJointAngles();
    
    std::vector<double> getJointsPosition();
    
    bool getCommState()
    {
        return cstate;
    }

    // int32_t getWrappedDXLDelta(int32_t current_pos, int32_t zero_pos) {
    //     int32_t delta = current_pos - 2048;
    //     if (delta > 2047) delta -= 2048;
    //     else if (delta < -2048) delta += 4096;
    //     return delta;
    // }

    bool isMoving() const
    {
        return is_moving_;
    }

    bool checkCollision() const
    {
        return false;
    }

private:
    uint8_t num_motors_;
    uint8_t *motor_ids_;
    std::string port_name_;
    int baudrate_;
    dynamixel::PortHandler *port_handler_;
    dynamixel::PacketHandler *packet_handler_;
    bool is_moving_;

    dynamixel::GroupBulkRead *groupBulkRead_;
    dynamixel::GroupBulkWrite *groupBulkWrite_;

    std::unique_ptr<dynamixel::GroupFastSyncRead> groupSyncReadPosition = nullptr;

    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    bool dxl_addparam_result = false;
    bool dxl_getdata_result = false;

    static constexpr uint16_t ADDR_TORQUE_ENABLE = 640;
    static constexpr uint16_t ADDR_GOAL_POSITION = 1160;
    static constexpr uint16_t ADDR_PRESENT_POSITION = 132;
    static constexpr uint8_t TORQUE_ENABLE = 1;
    static constexpr uint8_t TORQUE_DISABLE = 0;
    static constexpr uint8_t LEN_PRESENT_POSITION = 4;

    bool sync = false;
    bool cstate = false;
    std::vector<uint8_t> dxl_ids;
    bool first_value;

public:
    std::map<uint8_t, int32_t> present_positions;
    std::map<uint8_t, int32_t> present_velocities;
};
#endif // DYNAMIXEL_H
