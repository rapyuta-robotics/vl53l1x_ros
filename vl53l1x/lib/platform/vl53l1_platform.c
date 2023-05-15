/* Based on: https://github.com/pimoroni/vl53l1x-python */

#include "vl53l1_platform.h"
#include "vl53l1_api.h"
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <arpa/inet.h>
//#include "i2c.h"

static VL53L1_Error i2c_writeRegisterMulti(i2c* i2c, uint16_t reg, size_t count, void* pdata) {
    uint8_t buf[count + sizeof(reg)];

    reg = htons(reg);  // little endian to big endian

    memcpy(buf, &reg, sizeof(reg));
    memcpy(buf + sizeof(reg), pdata, count);

    return (libsoc_i2c_write(i2c, buf, count + sizeof(reg)) == EXIT_SUCCESS) ? VL53L1_ERROR_NONE
                                                                             : VL53L1_ERROR_CONTROL_INTERFACE;
}

static VL53L1_Error i2c_readRegisterMulti(i2c* i2c, uint16_t reg, size_t count, void* pdata) {
    reg = htons(reg);  // little endian to big endian

    if (libsoc_i2c_write(i2c, (uint8_t*) (&reg), sizeof(reg)) == EXIT_FAILURE) {
        return VL53L1_ERROR_CONTROL_INTERFACE;
    }

    return libsoc_i2c_read(i2c, pdata, count) == EXIT_SUCCESS ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t* pdata, uint32_t count) {
    return i2c_writeRegisterMulti(Dev->i2c_bus, index, count, pdata);
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t* pdata, uint32_t count) {
    return i2c_readRegisterMulti(Dev->i2c_bus, index, count, pdata);
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data) {
    return i2c_writeRegisterMulti(Dev->i2c_bus, index, 1, &data);
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data) {
    index = htons(index);  // little endian to big endian
    return (libsoc_i2c_write(Dev->i2c_bus, (uint8_t*) (&index), sizeof(index)) == EXIT_SUCCESS)
                   ? VL53L1_ERROR_NONE
                   : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

VL53L1_Error VL53L1_UpdateByte(VL53L1_DEV Dev, uint16_t index, uint8_t AndData, uint8_t OrData) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t* data) {
    return i2c_readRegisterMulti(Dev->i2c_bus, index, 1, data);
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t* data) {
    int ret = i2c_readRegisterMulti(Dev->i2c_bus, index, 2, data);
    if (ret == VL53L1_ERROR_NONE) {
        *data = ntohs(*data);  // big endian to little endian
    }

    return ret;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t* data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    return Status;
}

VL53L1_Error VL53L1_GetTickCount(uint32_t* ptick_count_ms) {
    VL53L1_Error status = VL53L1_ERROR_NONE;
    return status;
}

//#define trace_print(level, ...) \
//	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_PLATFORM, \
//	level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

//#define trace_i2c(...) \
//	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_NONE, \
//	VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)

VL53L1_Error VL53L1_GetTimerFrequency(int32_t* ptimer_freq_hz) {
    VL53L1_Error status = VL53L1_ERROR_NONE;
    return status;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t* pdev, int32_t wait_ms) {
    usleep(wait_ms * 1000);
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t* pdev, int32_t wait_us) {
    usleep(wait_us);
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitValueMaskEx(
        VL53L1_Dev_t* pdev, uint32_t timeout_ms, uint16_t index, uint8_t value, uint8_t mask, uint32_t poll_delay_ms) {
    uint8_t register_value = 0;

    VL53L1_Error status = VL53L1_ERROR_NONE;

    int32_t attempts = timeout_ms / poll_delay_ms;

    for (int32_t x = 0; x < attempts; x++) {
        status = VL53L1_RdByte(pdev, index, &register_value);
        if (status == VL53L1_ERROR_NONE && (register_value & mask) == value) {
            return VL53L1_ERROR_NONE;
        }
        usleep(poll_delay_ms * 1000);
    }

    return VL53L1_ERROR_TIME_OUT;
}
