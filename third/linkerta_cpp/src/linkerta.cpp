#include "linkerta.hpp"

// 移除这些全局变量声明
// std::vector<double> linkerta_offset;
// std::vector<int64_t> linkerta_negation;
// int calibration;

MasterArm::MasterArm(uint8_t num_motors, uint8_t *motor_ids, const std::string &port_name, int baudrate) : port_name_(port_name), baudrate_(baudrate), is_moving_(false), cstate(true), first_value(true)
{
    dxl_ids = {DXL_ID_1, DXL_ID_2, DXL_ID_3, DXL_ID_4, DXL_ID_5, DXL_ID_6, DXL_ID_7, DXL_ID_8, DXL_ID_9, DXL_ID_10, DXL_ID_11, DXL_ID_12, DXL_ID_13, DXL_ID_14};

    port_handler_ = dynamixel::PortHandler::getPortHandler(port_name.c_str());
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (port_handler_->openPort())
    {
        printf("Succeeded to open the port!\n");
    } else {
        printf("Failed to open the port!\n");
        // printf("Press any key to terminate...\n");
        // getchar();
        return;
    }

    if (port_handler_->setBaudRate(baudrate_))
    {
        printf("Succeeded to change the baudrate!\n");
    } else {
        printf("Failed to change the baudrate!\n");
        // printf("Press any key to terminate...\n");
        // getchar();
        return;
    }

    groupBulkRead_ = new GroupBulkRead(port_handler_, packet_handler_);
    groupBulkWrite_ = new GroupBulkWrite(port_handler_, packet_handler_);

    groupSyncReadPosition.reset(new dynamixel::GroupFastSyncRead(port_handler_, packet_handler_, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION));

    for (uint8_t id : dxl_ids)
    {
        dxl_addparam_result = groupBulkRead_->addParam(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupBulkRead addParam failed for Position\n", id);
        }

        // dxl_addparam_result = groupBulkRead.addParam(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
        // if (dxl_addparam_result != true) {
        //     fprintf(stderr, "[ID:%03d] groupBulkRead addParam failed for Velocity\n", id);
        // }

        // dxl_addparam_result = groupBulkRead.addParam(id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
        // if (dxl_addparam_result != true) {
        //     fprintf(stderr, "[ID:%03d] groupBulkRead addParam failed for Current\n", id);
        // }

        if (!groupSyncReadPosition->addParam(id))
        {
            fprintf(stderr, "Failed to add DXL ID %d to SyncRead group\n", id);
        }
    }

    printf("GroupBulkRead parameter setup complete.\n");

}

MasterArm::~MasterArm()
{
    delete groupBulkRead_;
    groupBulkRead_ = nullptr;

    delete groupBulkWrite_;
    groupBulkWrite_ = nullptr;

    for (uint8_t id : dxl_ids)
    {
    	packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
    }
    port_handler_->closePort();
}

std::vector<double> MasterArm::getJointAngles()
{
    if (sync == true)
    {
        std::cout << "Sync mode is enabled, using getSyncJointAngles instead." << std::endl;
    }

    std::vector<double> angles(dxl_ids.size());

    dxl_comm_result = groupBulkRead_->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("BulkRead txRxPacket failed: %s\n", packet_handler_->getTxRxResult(dxl_comm_result));
        cstate = false;
    } else {

#if 1
        for (uint8_t id : dxl_ids)
        {
            dxl_getdata_result = groupBulkRead_->isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            if (dxl_getdata_result)
            {
                present_positions[id] = groupBulkRead_->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
                angles[id] = present_positions[id] / 4096.0 * 360.0;
                // printf("[ID:%03d] PresPos:%d  ", id, present_positions[id]);
                
                cstate = true;
            }
            else
            {
                fprintf(stderr, "[ID:%03d] Failed to get Present Position data.\n", id);
                angles[id] = present_positions[id] / 4096.0 * 360.0;
                cstate = false;
            }

            // dxl_getdata_result = groupBulkRead.isAvailable(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
            // if (dxl_getdata_result) {
            //     present_velocities[id] = groupBulkRead.getData(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
            //     // printf("PresVel:%d\n", present_velocities[id]);
            // } else {
            //     fprintf(stderr, "[ID:%03d] Failed to get Present Velocity data.\n", id);
            //     present_velocities[id] = -999999; // Example error marker
            // }

            // dxl_getdata_result = groupBulkRead.isAvailable(id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
            // if (dxl_getdata_result) {
            //     present_currents[id] = groupBulkRead.getData(id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
            //     // printf("PresCur:%d\n", present_currents[id]);
            // } else {
            //     fprintf(stderr, "[ID:%03d] Failed to get Present Current data.\n", id);
            // }
        } // end for each id
#endif
    }
    return angles;
}

std::vector<double> MasterArm::getSyncJointAngles()
{
    sync = true;
    std::vector<double> angles(dxl_ids.size(), 0.0);

    dxl_comm_result = groupSyncReadPosition->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        fprintf(stderr, "SyncRead position failed: %s\n", packet_handler_->getTxRxResult(dxl_comm_result));
        cstate = false;
    }

    for (uint8_t id : dxl_ids)
    {
        if (groupSyncReadPosition->isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION))
        {
            present_positions[id] = groupSyncReadPosition->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            angles[id] = present_positions[id] / 4096.0 * 360.0;
            cstate = true;
        }
        else
        {
            fprintf(stderr, "Failed to read position for DXL ID %d\n", id);
            angles[id] = present_positions[id] / 4096.0 * 360.0;
            cstate = false;
        }
    }
    return angles;
}

std::vector<double> MasterArm::getJointsPosition()
{
    std::vector<double> joints_angles = getSyncJointAngles();
    
    // 简化函数，只返回原始角度值，标定逻辑在main.cpp中处理
    return joints_angles;
}