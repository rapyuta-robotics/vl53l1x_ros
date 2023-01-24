/*
 * STM VL53L1X ToF rangefinder driver for ROS
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under BSD 3-Clause License (available at https://opensource.org/licenses/BSD-3-Clause).
 *
 * Documentation used:
 * VL53L1X datasheet - https://www.st.com/resource/en/datasheet/vl53l1x.pdf
 * VL53L1X API user manual -
 * https://www.st.com/content/ccc/resource/technical/document/user_manual/group0/98/0d/38/38/5d/84/49/1f/DM00474730/files/DM00474730.pdf/jcr:content/translations/en.DM00474730.pdf
 *
 */

#include "vl53l1x.hpp"

Vl53l1x::Vl53l1x(
        ros::NodeHandle nh, int i2c_bus, int addr, std::string frame_id, std::mutex* mtx, struct gpiod_line* io_line) {
    _sensor_dev.i2c_bus = libsoc_i2c_init(i2c_bus, addr);
    _frame_id = frame_id;
    _nh = nh;
    _mtx = mtx;
    _io_line = io_line;

    ros::NodeHandle pnh("~");
    pnh.param("mode", _mode, 3);
    pnh.param("poll_rate", _poll_rate, 100.0);
    pnh.param("ignore_range_status", _ignore_range_status, false);
    pnh.param("timing_budget", _timing_budget, 0.1);
    pnh.param("offset", _offset, 0.0);
    pnh.param("field_of_view", _range_msg.field_of_view, 0.471239f);  // 27 deg, source: datasheet
    pnh.param("min_range", _range_msg.min_range, 0.0f);
    pnh.param("max_range", _range_msg.max_range, 4.0f);
    // pnh.getParam("pass_statuses", _pass_statuses);

    if (pnh.getParam("min_signal", _min_signal)) {
        CHECK_STATUS(VL53L1_SetLimitCheckValue(
                &_sensor_dev, VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, _min_signal * 65536));
    }

    if (pnh.getParam("max_sigma", _max_sigma)) {
        CHECK_STATUS(VL53L1_SetLimitCheckValue(
                &_sensor_dev, VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE, _max_sigma * 1000 * 65536));
    }

    // The minimum inter-measurement period must be longer than the timing budget + 4 ms (*)
    _inter_measurement_period = _timing_budget + 0.004;
};

Vl53l1x::~Vl53l1x(){};

void Vl53l1x::init(){};

void Vl53l1x::configure(){};

void Vl53l1x::sensor_prep() {
    // cnfiguration and verification
    std::lock_guard<std::mutex> lock(*_mtx);
    setEnable(1);
    VL53L1_DataInit(&_sensor_dev);
    VL53L1_StaticInit(&_sensor_dev);
    VL53L1_SetPresetMode(&_sensor_dev, VL53L1_PRESETMODE_AUTONOMOUS);

    // Print device info
    VL53L1_DeviceInfo_t device_info;
    CHECK_STATUS(VL53L1_GetDeviceInfo(&_sensor_dev, &device_info));
    ROS_INFO("VL53L1X: Device name: %." STR(VL53L1_DEVINFO_STRLEN) "s", device_info.Name);
    ROS_INFO("VL53L1X: Device type: %." STR(VL53L1_DEVINFO_STRLEN) "s", device_info.Type);
    ROS_INFO("VL53L1X: Product ID: %." STR(VL53L1_DEVINFO_STRLEN) "s", device_info.ProductId);
    ROS_INFO("VL53L1X: Type: %u Version: %u.%u", device_info.ProductType, device_info.ProductRevisionMajor,
            device_info.ProductRevisionMinor);

    // Setup sensor
    CHECK_STATUS(VL53L1_SetDistanceMode(&_sensor_dev, _mode));
    CHECK_STATUS(VL53L1_SetMeasurementTimingBudgetMicroSeconds(&_sensor_dev, round(_timing_budget * 1e6)));

    // Start sensor
    for (int i = 0; i < 100; i++) {
        CHECK_STATUS(
                VL53L1_SetInterMeasurementPeriodMilliSeconds(&_sensor_dev, round(_inter_measurement_period * 1e3)));
        _sensor_error = VL53L1_StartMeasurement(&_sensor_dev);
        if (_sensor_error == VL53L1_ERROR_INVALID_PARAMS) {
            _inter_measurement_period += 0.001;  // Increase inter_measurement_period to satisfy condition (*)
        } else
            break;
    }

    setEnable(0);

    // Check for errors after start
    if (_sensor_error != VL53L1_ERROR_NONE) {
        ROS_FATAL("VL53L1X: Can't start measurement: error %d", _sensor_error);
        ros::shutdown();
    }
}

void Vl53l1x::measureAndPublishTask() {
    ros::Publisher range_pub = _nh.advertise<sensor_msgs::Range>(_frame_id + "/range", 20);
    ros::Publisher data_pub = _nh.advertise<vl53l1x::MeasurementData>(_frame_id + "/data", 20);
    _range_msg.header.frame_id = _frame_id;
    ros::Rate r(_poll_rate);

    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
        _range_msg.header.stamp = ros::Time::now();
        std::lock_guard<std::mutex> lock(*_mtx);
        setEnable(1);

        // Check the data is ready
        VL53L1_GetMeasurementDataReady(&_sensor_dev, &_data_ready);
        if (!_data_ready) {
            setEnable(0);
            continue;
        }

        // Read measurement
        VL53L1_GetRangingMeasurementData(&_sensor_dev, &_measurement_data);
        VL53L1_ClearInterruptAndStartMeasurement(&_sensor_dev);

        // Publish measurement data
        _data_msg.header.stamp = _range_msg.header.stamp;
        _data_msg.signal = _measurement_data.SignalRateRtnMegaCps / 65536.0;
        _data_msg.ambient = _measurement_data.AmbientRateRtnMegaCps / 65536.0;
        _data_msg.effective_spad = _measurement_data.EffectiveSpadRtnCount / 256;
        _data_msg.sigma = _measurement_data.SigmaMilliMeter / 65536.0 / 1000.0;
        _data_msg.status = _measurement_data.RangeStatus;
        data_pub.publish(_data_msg);

        // Check measurement for validness
        if (!_ignore_range_status && std::find(_pass_statuses.begin(), _pass_statuses.end(),
                                             _measurement_data.RangeStatus) == _pass_statuses.end()) {
            char range_status[VL53L1_MAX_STRING_LENGTH];
            VL53L1_get_range_status_string(_measurement_data.RangeStatus, range_status);
            ROS_DEBUG("Range measurement status is not valid: %s", range_status);
            ros::spinOnce();
            setEnable(0);
            continue;
        }

        // Publish measurement
        _range_msg.range = _measurement_data.RangeMilliMeter / 1000.0 + _offset;
        range_pub.publish(_range_msg);

        setEnable(0);
    }

    // Release
    ROS_INFO("VL53L1X: stop ranging");
    VL53L1_StopMeasurement(&_sensor_dev);
};

void Vl53l1x::setEnable(uint8_t enable) {
    gpiod_line_set_value(_io_line, enable);
    usleep(500);
};
