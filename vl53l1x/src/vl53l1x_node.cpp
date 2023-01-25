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

#include <string>
#include <vector>
#include <ros/ros.h>
#include <mutex>

#include <gpiod.h>
#include <libsoc_i2c.h>
#include <boost/thread.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "vl53l1x");
    ros::NodeHandle nh, nh_priv("~");

    uint8_t xshut_pins[4] = {14, 15, 24, 27};
    int xshut_gpio_chip, i2c_bus, new_addr_index;
    struct gpiod_chip* gpio_chip;  // assumption: all the gpio lines are part of on gpio chip
    struct gpiod_line* io_line[4];
    std::mutex mtx;

    std::string sensor_frame_ids[4] = {
            "distance_sensor_front", "distance_sensor_rear", "distance_sensor_left", "distance_sensor_right"};

    nh_priv.param("i2c_bus", i2c_bus, 1);
    nh_priv.param("xshut_gpio_chip", xshut_gpio_chip, 0);
    nh_priv.param("new_addr_index", new_addr_index, 0x30);

    // Sequence for writing new slave address to the sensor(s)
    {
        gpio_chip = gpiod_chip_open_by_number(xshut_gpio_chip);
        for (int i = 0; i < 4; i++) {
            io_line[i] = gpiod_chip_get_line(gpio_chip, xshut_pins[i]);
            gpiod_line_request_output(io_line[i], "vl53l1x", 0);
            usleep(5000);
            gpiod_line_set_value(io_line[i], 0);
        }
        i2c* i2c = libsoc_i2c_init(i2c_bus, VL53L1_DEFAULT_ADDR);
        VL53L1_Dev_t sensor_dev[4];
        for (int i = 0; i < 4; i++) {
            sensor_dev[i].i2c_bus = i2c;
            gpiod_line_set_value(io_line[i], 1);
            VL53L1_software_reset(&sensor_dev[i]);
            VL53L1_WaitDeviceBooted(&sensor_dev[i]);
            VL53L1_SetDeviceAddress(&sensor_dev[i], ((new_addr_index + i) << 1));
            gpiod_line_set_value(io_line[i], 0);
        }
        libsoc_i2c_free(i2c);  // rlease the temporary i2c instance
    }

    std::vector<Vl53l1x*> sensors;
    for (int i = 0; i < 4; i++) {
        sensors.emplace_back(new Vl53l1x(nh, i2c_bus, new_addr_index + i, sensor_frame_ids[i], &mtx, io_line[i]));
    }

    for (auto sensor : sensors) {
        sensor->sensor_prep();
        boost::thread(boost::bind(&Vl53l1x::measureAndPublishTask, sensor));
    }

    ros::waitForShutdown();
}
