#pragma once

#include <string>
#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <vl53l1x/MeasurementData.h>

#include "vl53l1_api.h"
#include <gpiod.h>

#define xSTR(x) #x
#define STR(x) xSTR(x)

#define CHECK_STATUS(func)                                          \
    {                                                               \
        VL53L1_Error status = func;                                 \
        if (status != VL53L1_ERROR_NONE) {                          \
            ROS_WARN("VL53L1X: Error %d on %s", status, STR(func)); \
        }                                                           \
    }

const uint8_t VL53L1_DEFAULT_ADDR = 0x29;

class Vl53l1x {
public:
    Vl53l1x(ros::NodeHandle nh, int i2c_bus, int addr, std::string frame_id, std::mutex* mtx,
            struct gpiod_line* io_line);
    ~Vl53l1x();

    void init();
    void configure();
    void measureAndPublishTask();
    void sensor_prep();

private:
    i2c* _i2c_bus;
    int _address;

    std::string _frame_id;
    int _mode;
    double _poll_rate, _timing_budget, _offset;
    double _inter_measurement_period;
    bool _ignore_range_status;
    double _min_signal;
    double _max_sigma;
    uint8_t _data_ready = 0;
    const std::vector<int> _pass_statuses{VL53L1_RANGESTATUS_RANGE_VALID,
            VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL, VL53L1_RANGESTATUS_RANGE_VALID_MERGED_PULSE};

    ros::NodeHandle _nh;
    ros::NodeHandle _pnh;
    sensor_msgs::Range _range_msg;
    vl53l1x::MeasurementData _data_msg;

    VL53L1_Dev_t _sensor_dev;
    VL53L1_Error _sensor_error;
    VL53L1_DeviceInfo_t _device_info;
    VL53L1_RangingMeasurementData_t _measurement_data;

    std::mutex* _mtx;
    struct gpiod_line* _io_line;

    // ref: https://qiita.com/srs/items/4a658522a7f5dea5b83f
};
